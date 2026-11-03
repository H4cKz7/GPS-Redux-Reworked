#include "GPS.h"

#include <vector>

void GPS::Run()
{
	logger = util::Logger(cfg.LOGFILE_ENABLED);

	std::ranges::fill(pathNodesToStream, 1);
	std::ranges::fill(pathNodes, -1);

	ApplyPatches();

	if (GetModuleHandleA("samp.dll"))
	{
		// SA-MP installs its own hooks on the game; hooking too early gets our hooks overwritten.
		// Register later from a helper thread instead.
		HANDLE h = CreateThread(nullptr, 0, &GPS::DeferredRegisterThread, this, 0, nullptr);
		if (h)
			CloseHandle(h);
		else
			RegisterEvents();
	}
	else
	{
		// Singleplayer: unchanged behaviour, register immediately.
		RegisterEvents();
	}
}

void GPS::RegisterEvents()
{
	plugin::Events::gameProcessEvent += [this]() { this->GameEventHandle(); };

	plugin::Events::drawRadarOverlayEvent += [this]() { this->DrawRadarOverlayHandle(); };

	plugin::Events::drawRadarEvent += [this]() { this->DrawHudEventHandle(); };
}

DWORD WINAPI GPS::DeferredRegisterThread(LPVOID param)
{
	GPS *self = static_cast<GPS *>(param);

	Sleep(GPS_SAMP_HOOK_DELAY_MS);

	// On a slow first launch the game can still be loading after the delay. Wait (up to a minute) until the
	// player exists before hooking in, so our callbacks never run against a half-initialised game.
	for (int waited = 0; waited < 120 && !FindPlayerPed(0); ++waited)
		Sleep(500);

	// SA-MP may have overwritten the patches we applied at load time, so re-apply them.
	self->ApplyPatches();

	self->RegisterEvents();

	return 0;
}

void GPS::ApplyPatches()
{
	plugin::patch::SetPointer(0x44DE3C, pathNodesToStream.data());
	plugin::patch::SetPointer(0x450D03, pathNodesToStream.data());
	plugin::patch::SetPointer(0x451782, pathNodes.data());
	plugin::patch::SetPointer(0x451904, pathNodes.data());
	plugin::patch::SetPointer(0x451AC3, pathNodes.data());
	plugin::patch::SetPointer(0x451B33, pathNodes.data());
	plugin::patch::SetUInt(0x4518F8, 50000);
	plugin::patch::SetUInt(0x4519B0, 49950);
}

// The 64 path-node files (8x8 areas of 750x750 units) are streaming resources 25511..25574.
// Under singleplayer the game streams them as the player moves and the mod's patches keep them all loaded.
// Under SA-MP only the areas around the spawn point get loaded, so a route can't be found anywhere else
// (or snaps to far away nodes). Requesting every area keeps the whole road network available.
void GPS::RequestAllPathAreas()
{
	// 0x6 = GAME_REQUIRED | MISSION_REQUIRED, so the streaming system doesn't evict them again.
	for (int i = 0; i < 64; ++i)
		CStreaming::RequestModel(25511 + i, 0x6);
}

// True when every path-node area (streaming models 25511..25574) is fully loaded. 1 = LOADSTATE_LOADED.
bool GPS::PathAreasLoaded()
{
	for (int i = 0; i < 64; ++i)
	{
		if (CStreaming::ms_aInfoForModel[25511 + i].m_nLoadState != 1)
			return false;
	}
	return true;
}

// Returns true when running inside SA-MP. samp.dll may load after this ASI, so it is checked lazily.
bool GPS::IsSAMP()
{
	static HMODULE samp = nullptr;
	if (!samp)
		samp = GetModuleHandleA("samp.dll");
	return samp != nullptr;
}

// Returns the active waypoint trace, or nullptr if the index is stale or invalid.
static tRadarTrace *GetTargetTrace()
{
	const int idx = FrontEndMenuManager.m_nTargetBlipIndex;
	if (!idx)
		return nullptr;

	const unsigned lo = LOWORD(idx);
	if (lo >= 175)
		return nullptr;

	tRadarTrace *t = &CRadar::ms_RadarTrace[lo];
	if (t->m_nCounter != HIWORD(idx) || !t->m_nBlipDisplay)
		return nullptr;

	return t;
}

// Must be called on the game thread. Snapshots everything the worker thread needs,
// so the worker never dereferences the player's vehicle (which SA-MP can destroy at any time).
bool GPS::captureSearchParams(CVector &startPosn, bool &respectLanes, bool &waterNav)
{
	if (!player || !player->m_pVehicle)
		return false;

	const auto subClass = player->m_pVehicle->m_nVehicleSubClass;
	// SA-MP: use the vehicle's position (the ped's own position can lag behind in multiplayer).
	// Singleplayer keeps using the ped position as before.
	startPosn = IsSAMP() ? CVector(player->m_pVehicle->GetPosition()) : CVector(player->GetPosition());
	// Respect rules of traffic (only if in valid vehicle & enabled in config)
	respectLanes = subClass != VEHICLE_BOAT && subClass != VEHICLE_BMX && cfg.RESPECT_LANE_DIRECTION;
	// Whether to do water navigation
	waterNav = subClass == VEHICLE_BOAT && cfg.ENABLE_WATER_GPS;
	return true;
}

// The game's path search keeps its working data in shared global arrays, so two searches must never run at the
// same time (the waypoint and mission searches run on separate threads). Overlapping searches corrupt that data
// and can leave every later search returning an empty route.
static std::mutex g_searchMutex;

// How the destination gets resolved (everything below is only touched while holding g_searchMutex).
//
// The game picks the road node closest to the waypoint on its own, and that lookup also weighs the node's height.
// A waypoint on Mount Chiliad has a placeholder/ground Z and sits off-road, so the game often snaps to a node that is
// not the nearest one on the map, or to one the player's road network can't reach (empty route). Instead, the nearest
// vehicle nodes are found here by X/Y distance only and the route is searched towards the nearest one that is
// actually reachable. The result is remembered per waypoint so this only happens once, not every frame.
namespace
{
	struct DestFix
	{
		bool used = false;
		bool failed = false; // nothing was reachable; don't retry until 'tick' + 3s has passed
		bool lanes = false;
		float ox = 0.0f, oy = 0.0f;			   // original destination (X/Y)
		float dx = 0.0f, dy = 0.0f, dz = 0.0f; // destination that actually gets searched (a road node)
		DWORD tick = 0;
	};
	DestFix g_destFix[4];
	unsigned g_destFixNext = 0;

	struct NearNode
	{
		float d2; // squared X/Y distance to the waypoint
		float x, y, z;
	};
} // namespace

// Finds up to 'maxCount' vehicle path nodes closest to 'target' by X/Y distance, nearest first. Nodes closer than
// 'minSpacing' to one that was already picked are skipped, so the list offers genuinely different places to
// try (if the nearest node is on an unreachable island, its neighbours are on that island too).
static void FindNearestVehicleNodes2D(const CVector &target, std::vector<NearNode> &out, size_t maxCount,
									  float minSpacing)
{
	out.clear();

	std::vector<NearNode> all;
	all.reserve(16384);

	for (int area = 0; area < NUM_PATH_MAP_AREAS; ++area)
	{
		CPathNode *nodes = ThePaths.m_pPathNodes[area];
		if (!nodes)
			continue;

		// Vehicle nodes come first in each area, ped nodes after them.
		unsigned count = ThePaths.m_dwNumVehicleNodes[area];
		if (count > ThePaths.m_dwNumNodes[area])
			count = ThePaths.m_dwNumNodes[area];

		for (unsigned i = 0; i < count; ++i)
		{
			const CVector p = nodes[i].GetNodeCoors();
			if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || std::fabs(p.x) > GPS_WORLD_LIMIT ||
				std::fabs(p.y) > GPS_WORLD_LIMIT)
				continue;

			const float dx = p.x - target.x;
			const float dy = p.y - target.y;
			all.push_back({dx * dx + dy * dy, p.x, p.y, p.z});
		}
	}

	const size_t keep = std::min<size_t>(all.size(), 400);
	std::partial_sort(all.begin(), all.begin() + keep, all.end(),
					  [](const NearNode &a, const NearNode &b) { return a.d2 < b.d2; });

	const float spacing2 = minSpacing * minSpacing;
	for (size_t i = 0; i < keep && out.size() < maxCount; ++i)
	{
		bool tooClose = false;
		for (const NearNode &o : out)
		{
			const float dx = all[i].x - o.x;
			const float dy = all[i].y - o.y;
			if (dx * dx + dy * dy < spacing2)
			{
				tooClose = true;
				break;
			}
		}
		if (!tooClose)
			out.push_back(all[i]);
	}
}

void GPS::calculatePath(const CVector &startPosn, const CVector &destPosn, bool respectLanes, bool waterNav,
						short &nodesCount, CNodeAddress *resultNodes, float &gpsDistance)
{
	std::lock_guard<std::mutex> searchLock(g_searchMutex);

	auto search = [&](const CVector &dest, bool lanes) {
		nodesCount = 0;
		gpsDistance = 0.0f;
		ThePaths.DoPathSearch(0, startPosn, CNodeAddress(), dest, resultNodes, &nodesCount, MAX_NODE_POINTS,
							  &gpsDistance, 999999.0f, NULL, 999999.0f, lanes, CNodeAddress(), false, waterNav);
	};

	// Boats use water nodes, which can't be told apart from road nodes here: keep the game's own lookup for them.
	if (waterNav)
	{
		search(destPosn, respectLanes);
		return;
	}

	const DWORD now = GetTickCount();

	// Already resolved this destination? Search straight to the remembered road node.
	DestFix *fix = nullptr;
	for (auto &f : g_destFix)
	{
		if (f.used && std::fabs(f.ox - destPosn.x) <= 1.0f && std::fabs(f.oy - destPosn.y) <= 1.0f)
		{
			fix = &f;
			break;
		}
	}
	if (fix && !fix->failed)
	{
		search(CVector(fix->dx, fix->dy, fix->dz), respectLanes && fix->lanes);
		if (nodesCount > 0)
			return;
		fix->used = false; // stale (e.g. the player moved to another part of the map), resolve again
		fix = nullptr;
	}

	// Recently found nothing reachable for this destination: behave like the original mod and let the game decide.
	if (fix && fix->failed && now - fix->tick < 3000)
	{
		search(destPosn, respectLanes);
		return;
	}

	std::vector<NearNode> cands;
	FindNearestVehicleNodes2D(destPosn, cands, 10, 40.0f);

	// Nearest reachable road node wins. Lane rules can cut off remote one-way/dead-end roads, so if nothing is
	// reachable with them, try again without.
	for (int pass = 0; pass < 2; ++pass)
	{
		const bool lanes = pass == 0 ? respectLanes : false;
		if (pass == 1 && !respectLanes)
			break;

		for (const NearNode &c : cands)
		{
			search(CVector(c.x, c.y, c.z), lanes);
			if (nodesCount > 0)
			{
				if (!fix)
					fix = &g_destFix[g_destFixNext++ % 4];
				fix->used = true;
				fix->failed = false;
				fix->lanes = lanes;
				fix->ox = destPosn.x;
				fix->oy = destPosn.y;
				fix->dx = c.x;
				fix->dy = c.y;
				fix->dz = c.z;
				fix->tick = now;
				return;
			}
		}
	}

	// Nothing reachable: remember that (so we don't retry every frame) and fall back to the game's own lookup.
	if (!fix)
		fix = &g_destFix[g_destFixNext++ % 4];
	fix->used = true;
	fix->failed = true;
	fix->ox = destPosn.x;
	fix->oy = destPosn.y;
	fix->tick = now;
	search(destPosn, respectLanes);
}

void GPS::requestTargetPath(CVector destPosn)
{
	// The waypoint was set/moved: drop the old route right away, and make sure a search that is still running
	// for the previous destination can't publish its (now wrong) route afterwards.
	// Only X/Y are compared: the game adjusts the waypoint's Z as the ground loads, which is not a new waypoint.
	const float moveX = destPosn.x - lastRequestedDest.x;
	const float moveY = destPosn.y - lastRequestedDest.y;
	if (!hasRequestedDest || (moveX * moveX + moveY * moveY) > 1.0f)
	{
		lastRequestedDest = destPosn;
		hasRequestedDest = true;
		++targetGeneration;

		std::lock_guard<std::mutex> lock(pathMutex);
		targetNodesCount = 0;
		targetDistance = 0.0f;
	}

	if (targetFuture.valid() && targetFuture.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
		return;

	CVector startPosn;
	bool respectLanes = false, waterNav = false;
	if (!captureSearchParams(startPosn, respectLanes, waterNav))
		return;

	const unsigned generation = targetGeneration.load();
	targetFuture = std::async(std::launch::async, [this, generation, startPosn, destPosn, respectLanes, waterNav]() {
		short nodesCountTemp = 0;
		float distanceTemp = 0.0f;
		std::array<CNodeAddress, MAX_NODE_POINTS> nodesTemp{};

		this->calculatePath(startPosn, destPosn, respectLanes, waterNav, nodesCountTemp, nodesTemp.data(),
							distanceTemp);

		// Never trust the node count blindly, it is used as an array index later.
		if (nodesCountTemp < 0 || nodesCountTemp > MAX_NODE_POINTS)
			nodesCountTemp = 0;

		std::lock_guard<std::mutex> lock(pathMutex);
		// The waypoint changed while this search was running: the result belongs to the old destination.
		if (generation != targetGeneration.load())
			return;
		targetNodesCount = nodesCountTemp;
		targetDistance = distanceTemp;
		std::copy(nodesTemp.begin(), nodesTemp.end(), t_ResultNodes.begin());
	});
}

void GPS::requestMissionPath(CVector destPosn)
{
	if (missionFuture.valid() && missionFuture.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
		return;

	CVector startPosn;
	bool respectLanes = false, waterNav = false;
	if (!captureSearchParams(startPosn, respectLanes, waterNav))
		return;

	missionFuture = std::async(std::launch::async, [this, startPosn, destPosn, respectLanes, waterNav]() {
		short nodesCountTemp = 0;
		float distanceTemp = 0.0f;
		std::array<CNodeAddress, MAX_NODE_POINTS> nodesTemp{};

		this->calculatePath(startPosn, destPosn, respectLanes, waterNav, nodesCountTemp, nodesTemp.data(),
							distanceTemp);

		if (nodesCountTemp < 0 || nodesCountTemp > MAX_NODE_POINTS)
			nodesCountTemp = 0;

		std::lock_guard<std::mutex> lock(pathMutex);
		missionNodesCount = nodesCountTemp;
		missionDistance = distanceTemp;
		std::copy(nodesTemp.begin(), nodesTemp.end(), m_ResultNodes.begin());
	});
}

// Events

void GPS::DrawRadarOverlayHandle()
{
	if (!util::NavEnabled(this->cfg, player))
		return;

	if (renderTargetRoute)
	{
		std::lock_guard<std::mutex> lock(pathMutex);
		this->renderPath(targetTracePos, -1, false, targetNodesCount, t_ResultNodes.data(), targetDistance,
						 t_LineVerts.data());
	}

	if (renderMissionRoute)
	{
		try
		{
			this->renderMissionTrace(mTrace);
		}
		catch (const std::exception &e)
		{
			logger.Log(e.what());
			renderMissionRoute = false;
		}
	}
}

void GPS::GameEventHandle()
{
	player = FindPlayerPed(0);
	if (!player)
	{
		renderTargetRoute = false;
		renderMissionRoute = false;
		mTrace = nullptr;
		return;
	}

	const bool samp = IsSAMP();

	// SA-MP only: make sure all path-node areas are loaded (re-requested every few seconds).
	if (samp)
	{
		const DWORD nowReq = GetTickCount();
		if (nowReq - lastPathAreaRequest >= 3000)
		{
			lastPathAreaRequest = nowReq;
			RequestAllPathAreas();
		}
	}

	renderTargetRoute = FrontEndMenuManager.m_nTargetBlipIndex == 0 ? false : true;

	if (!mTrace)
		renderMissionRoute = false;
	else
	{
		renderMissionRoute = mTrace->m_bInUse;
		if (mTrace->m_nBlipDisplay < 2 ||
			DistanceBetweenPoints(player->GetPosition(), mTrace->m_vecPos) <= cfg.DISABLE_PROXIMITY)
		{
			renderMissionRoute = false;
		}

		if (!renderMissionRoute)
			mTrace = nullptr;
	}

	if (!util::NavEnabled(this->cfg, player))
	{
		renderMissionRoute = false;
		return;
	}

	// Clear the waypoint when the player is close enough to it.
	// Only X/Y are compared: the waypoint's Z is often a placeholder (not the real ground height), so a 3D distance
	// would never get small enough on hills/low ground and the marker would stay. SA-MP uses the vehicle position
	// because the ped's own position can lag behind in multiplayer.
	if (tRadarTrace *t = GetTargetTrace())
	{
		const CVector here =
			(samp && player->m_pVehicle) ? CVector(player->m_pVehicle->GetPosition()) : CVector(player->GetPosition());
		const float dx = here.x - t->m_vecPos.x;
		const float dy = here.y - t->m_vecPos.y;
		if (sqrtf(dx * dx + dy * dy) <= cfg.DISABLE_PROXIMITY)
		{
			CRadar::ClearBlip(FrontEndMenuManager.m_nTargetBlipIndex);
			FrontEndMenuManager.m_nTargetBlipIndex = 0;
			renderTargetRoute = false;
		}
	}

	// SA-MP: don't search until the road data has actually finished streaming in.
	if (samp && !pathAreasReady)
	{
		if (!PathAreasLoaded())
		{
			renderTargetRoute = false;
			return;
		}
		pathAreasReady = true;
	}

	// Request path to the waypoint
	if (tRadarTrace *t = GetTargetTrace())
	{
		targetTracePos = t->m_vecPos;
		this->requestTargetPath(targetTracePos);
		renderTargetRoute = true;
	}
	else
	{
		hasRequestedDest = false;
	}

	// Mission objective routes are singleplayer only. In SA-MP only the waypoint route is used.
	if (samp)
	{
		mTrace = nullptr;
		renderMissionRoute = false;
		return;
	}

	try
	{
		if (CTheScripts::IsPlayerOnAMission())
		{
			float bestDist = std::numeric_limits<float>::infinity();
			tRadarTrace *bestTrace = nullptr;

			for (int i = 0; i < 175; ++i)
			{
				tRadarTrace *trace = &CRadar::ms_RadarTrace[i];
				if (trace && trace->m_nRadarSprite == 0 && trace->m_nBlipDisplay > 1)
				{
					float d = DistanceBetweenPoints(player->GetPosition(), trace->m_vecPos);
					if (d < bestDist)
					{
						bestDist = d;
						bestTrace = trace;
					}
				}
			}

			mTrace = bestTrace;
			renderMissionRoute = bestTrace != nullptr;
		}
		else
		{
			renderMissionRoute = false;
		}
	}
	catch (const std::exception &e)
	{
		logger.Log(e.what());
		renderMissionRoute = false;
	}
}

void GPS::DrawHudEventHandle()
{
	if (!cfg.ENABLE_DISTANCE_TEXT)
		return;

	if (!NavEnabled(this->cfg, player))
		return;

	if (renderMissionRoute && mTrace)
	{
		CFont::SetOrientation(ALIGN_CENTER);
		CFont::SetColor(SetupColor(this->mTrace->m_nColour, this->mTrace->m_bFriendly, cfg));
		CFont::SetBackground(false, false);
		CFont::SetWrapx(500.0f);
		CFont::SetScale(0.3f * static_cast<float>(RsGlobal.maximumWidth) / 640.0f,
						0.6f * static_cast<float>(RsGlobal.maximumHeight) / 448.0f);
		CFont::SetFontStyle(FONT_SUBTITLES);
		CFont::SetProportional(true);
		CFont::SetDropShadowPosition(1);
		CFont::SetDropColor(CRGBA(0, 0, 0, 180));

		CVector2D point;
		CRadar::TransformRadarPointToScreenSpace(point, CVector2D(0.0f, -1.0f));
		CFont::PrintString(
			point.x, point.y + 8.0f * static_cast<float>(RsGlobal.maximumHeight) / 448.0f,
			(char *)util::makeDist(DistanceBetweenPoints(FindPlayerCoors(0), destVec), cfg.DISTANCE_UNITS).c_str());
	}

	if (renderTargetRoute)
	{
		tRadarTrace *t = GetTargetTrace();
		if (!t)
			return;

		CFont::SetOrientation(ALIGN_CENTER);
		CFont::SetColor(cfg.GPS_LINE_CLR);

		CFont::SetBackground(false, false);
		CFont::SetWrapx(500.0f);
		CFont::SetScale(0.3f * static_cast<float>(RsGlobal.maximumWidth) / 640.0f,
						0.6f * static_cast<float>(RsGlobal.maximumHeight) / 448.0f);
		CFont::SetFontStyle(FONT_SUBTITLES);
		CFont::SetProportional(true);
		CFont::SetDropShadowPosition(1);
		CFont::SetDropColor(CRGBA(0, 0, 0, 180));

		CVector2D point;
		CRadar::TransformRadarPointToScreenSpace(point, CVector2D(0.0f, 1.0f));
		CFont::PrintString(
			point.x, point.y - 20.0f * static_cast<float>(RsGlobal.maximumHeight) / 448.0f,
			(char *)util::makeDist(DistanceBetweenPoints(CVector(player->GetPosition()), CVector(t->m_vecPos)),
								   cfg.DISTANCE_UNITS)
				.c_str());
	}
}

// Rendering

void GPS::renderPath(CVector tracePos, short color, bool friendly, short &nodesCount, CNodeAddress *resultNodes,
					 float &gpsDistance, RwIm2DVertex *lineVerts)
{
	if (nodesCount <= 0)
	{
		return;
	}

	// The count is used as an array index below, so never let it exceed the buffers.
	const int count = nodesCount > MAX_NODE_POINTS ? MAX_NODE_POINTS : nodesCount;

	// Pass 1: world -> screen for every node. Nodes that can't be read, or have impossible coordinates, are
	// flagged so they are never drawn (they used to produce long streaks across the radar).
	float prevX = 0.0f, prevY = 0.0f;
	for (int i = 0; i < count; i++)
	{
		tmpNodeOk[i] = false;
		tmpSegmentOk[i] = false;
		tmpNodePoints[i] = CVector2D(0.0f, 0.0f);

		currentNode = ThePaths.GetPathNode(resultNodes[i]);
		if (!currentNode)
			continue;

		nodePosn = currentNode->GetNodeCoors();
		if (!std::isfinite(nodePosn.x) || !std::isfinite(nodePosn.y) || std::fabs(nodePosn.x) > GPS_WORLD_LIMIT ||
			std::fabs(nodePosn.y) > GPS_WORLD_LIMIT)
			continue;

		tmpNodeOk[i] = true;

		// Only join this node to the previous one when they are a believable distance apart.
		if (i > 0 && tmpNodeOk[i - 1])
		{
			const float gx = nodePosn.x - prevX;
			const float gy = nodePosn.y - prevY;
			tmpSegmentOk[i - 1] = (gx * gx + gy * gy) <= (GPS_MAX_SEGMENT_LENGTH * GPS_MAX_SEGMENT_LENGTH);
		}
		prevX = nodePosn.x;
		prevY = nodePosn.y;

		CRadar::TransformRealWorldPointToRadarSpace(tmpPoint, CVector2D(nodePosn.x, nodePosn.y));
		if (!FrontEndMenuManager.m_bDrawRadarOrMap)
		{
			CRadar::TransformRadarPointToScreenSpace(tmpNodePoints[i], tmpPoint);
		}
		else
		{
			CRadar::LimitRadarPoint(tmpPoint);
			CRadar::TransformRadarPointToScreenSpace(tmpNodePoints[i], tmpPoint);
			tmpNodePoints[i].x *= static_cast<float>(RsGlobal.maximumWidth) / 640.0f;
			tmpNodePoints[i].y *= static_cast<float>(RsGlobal.maximumHeight) / 448.0f;
			CRadar::LimitToMap(&tmpNodePoints[i].x, &tmpNodePoints[i].y);
		}
	}

	// Screen-space bounds of the radar. Only set when the scissor test clips the drawing to the radar.
	bool cull = false;
	float clipMinX = 0.0f, clipMaxX = 0.0f, clipMinY = 0.0f, clipMaxY = 0.0f;
	if (!FrontEndMenuManager.m_bDrawRadarOrMap &&
		reinterpret_cast<D3DCAPS9 const *>(RwD3D9GetCaps())->RasterCaps & D3DPRASTERCAPS_SCISSORTEST)
	{
		RECT rect;
		CVector2D posn;
		CRadar::TransformRadarPointToScreenSpace(posn, CVector2D(-1.0f, -1.0f));
		rect.left = static_cast<LONG>(posn.x + 2.0f);
		rect.bottom = static_cast<LONG>(posn.y - 2.0f);
		CRadar::TransformRadarPointToScreenSpace(posn, CVector2D(1.0f, 1.0f));
		rect.right = static_cast<LONG>(posn.x - 2.0f);
		rect.top = static_cast<LONG>(posn.y + 2.0f);
		reinterpret_cast<IDirect3DDevice9 *>(GetD3DDevice())->SetRenderState(D3DRS_SCISSORTESTENABLE, TRUE);
		reinterpret_cast<IDirect3DDevice9 *>(GetD3DDevice())->SetScissorRect(&rect);

		clipMinX = static_cast<float>(std::min(rect.left, rect.right));
		clipMaxX = static_cast<float>(std::max(rect.left, rect.right));
		clipMinY = static_cast<float>(std::min(rect.top, rect.bottom));
		clipMaxY = static_cast<float>(std::max(rect.top, rect.bottom));
		cull = clipMaxX > clipMinX && clipMaxY > clipMinY;
	}

	RwRenderStateSet(rwRENDERSTATETEXTURERASTER, NULL);

	const CRGBA vColor = SetupColor(color, friendly, cfg);

	// 'cull' stays false when the full screen map is open, so no visibility optimization is applied there.

	// The line is a triangle strip. Quads that are skipped (off the radar, or invalid) must not stay inside the same
	// strip: the strip would then connect the quads on either side of the gap, drawing a long streak across the
	// radar. So the strip is submitted and restarted every time there is a gap.
	vertIndex = 0;
	auto flushStrip = [&]() {
		if (vertIndex >= 4)
			RwIm2DRenderPrimitive(rwPRIMTYPETRISTRIP, lineVerts, vertIndex);
		vertIndex = 0;
	};

	// Create vertices for nodes
	for (int i = 0; i < count - 1; i++)
	{
		if (!tmpSegmentOk[i])
		{
			flushStrip();
			continue;
		}

		dir = tmpNodePoints[i + 1] - tmpNodePoints[i]; // Direction between current node to next
													   // node

		// Both nodes at the same spot: nothing to draw, and the next quad starts at the same point anyway.
		if (dir.x * dir.x + dir.y * dir.y < 0.0001f)
			continue;

		// Only set up vertices for segments that touch the radar. This must test the segment's bounding box, not just
		// its end points: on sparse roads (Mount Chiliad, desert) a single long segment can cross the radar while
		// both of its nodes are outside it, and skipping that segment made the line vanish or break up.
		const float margin = cfg.GPS_LINE_WIDTH + 2.0f;
		if (cull && (std::max(tmpNodePoints[i].x, tmpNodePoints[i + 1].x) + margin < clipMinX ||
					 std::min(tmpNodePoints[i].x, tmpNodePoints[i + 1].x) - margin > clipMaxX ||
					 std::max(tmpNodePoints[i].y, tmpNodePoints[i + 1].y) + margin < clipMinY ||
					 std::min(tmpNodePoints[i].y, tmpNodePoints[i + 1].y) - margin > clipMaxY))
		{
			flushStrip();
			continue;
		}

		angle = atan2(dir.y, dir.x); // Convert direction to angle

		if (!FrontEndMenuManager.m_bDrawRadarOrMap)
		{
			shift[0].x = cosf(angle - M_PI_2) * cfg.GPS_LINE_WIDTH;
			shift[0].y = sinf(angle - M_PI_2) * cfg.GPS_LINE_WIDTH;
			shift[1].x = cosf(angle + M_PI_2) * cfg.GPS_LINE_WIDTH;
			shift[1].y = sinf(angle + M_PI_2) * cfg.GPS_LINE_WIDTH;
		}
		else
		{
			float mp = FrontEndMenuManager.m_fMapZoom - 140.0f;
			if (mp < 140.0f)
				mp = 140.0f;
			else if (mp > 960.0f)
				mp = 960.0f;
			mp = mp / 960.0f + 0.4f;
			shift[0].x = cosf(angle - M_PI_2) * cfg.GPS_LINE_WIDTH * mp;
			shift[0].y = sinf(angle - M_PI_2) * cfg.GPS_LINE_WIDTH * mp;
			shift[1].x = cosf(angle + M_PI_2) * cfg.GPS_LINE_WIDTH * mp;
			shift[1].y = sinf(angle + M_PI_2) * cfg.GPS_LINE_WIDTH * mp;
		}

		util::Setup2dVertex(				 //
			lineVerts[vertIndex + 0],		 //
			tmpNodePoints[i].x + shift[0].x, // CurrentNode*
			tmpNodePoints[i].y + shift[0].y, //
			vColor);

		util::Setup2dVertex(				 //
			lineVerts[vertIndex + 1],		 //
			tmpNodePoints[i].x + shift[1].x, // CurrentNode - CurrentNode*
			tmpNodePoints[i].y + shift[1].y, //
			vColor);

		util::Setup2dVertex(					 // NextNode*
			lineVerts[vertIndex + 2],			 //    |
			tmpNodePoints[i + 1].x + shift[0].x, // CurrentNode - CurrentNode
			tmpNodePoints[i + 1].y + shift[0].y, //
			vColor);

		util::Setup2dVertex(lineVerts[vertIndex + 3],			 // NextNode - NextNode*
							tmpNodePoints[i + 1].x + shift[1].x, //    | |
							tmpNodePoints[i + 1].y + shift[1].y, // CurrentNode - CurrentNode
							vColor);

		vertIndex += 4;
	}

	flushStrip();

	if (!FrontEndMenuManager.m_bDrawRadarOrMap &&
		reinterpret_cast<D3DCAPS9 const *>(RwD3D9GetCaps())->RasterCaps & D3DPRASTERCAPS_SCISSORTEST)
	{
		reinterpret_cast<IDirect3DDevice9 *>(GetD3DDevice())->SetRenderState(D3DRS_SCISSORTESTENABLE, FALSE);
	}

	CPathNode *firstNode = ThePaths.GetPathNode(resultNodes[0]);
	if (firstNode && player)
		gpsDistance += DistanceBetweenPoints(player->GetPosition(), firstNode->GetNodeCoors());
}

void GPS::renderMissionTrace(tRadarTrace *trace)
{
	// this->Log("Found mission objective blip.");
	if (!trace)
		return;

	if (trace->m_nBlipDisplay < 2)
	{
		renderMissionRoute = false;
		return;
	}

	switch (trace->m_nBlipType)
	{
	case 1:
		if (cfg.ENABLE_MOVING)
		{
			CVehicle *veh = CPools::GetVehicle(trace->m_nEntityHandle);
			if (!veh)
			{
				renderMissionRoute = false;
				return;
			}
			destVec = veh->GetPosition();
		}
		else
		{
			renderMissionRoute = false;
			return;
		}
		break;
	case 2:
		if (cfg.ENABLE_MOVING)
		{
			CPed *ped = CPools::GetPed(trace->m_nEntityHandle);
			if (!ped)
			{
				renderMissionRoute = false;
				return;
			}
			destVec = ped->GetPosition();
		}
		else
		{
			renderMissionRoute = false;
			return;
		}
		break;
	case 3: {
		CObject *obj = CPools::GetObject(trace->m_nEntityHandle);
		if (!obj)
		{
			renderMissionRoute = false;
			return;
		}
		destVec = obj->GetPosition();
		break;
	}
	case 6: // Searchlights
	case 8: // Airstripts
	case 0: // NONE???
		return;
	case 7: // Pickups
		renderMissionRoute = false;
		logger.Log("Pickup detected. Not providing GPS navigation!");
		return;
	default:
		destVec = trace->m_vecPos;
		break;
	}

	// this->Log("DestVec: " + std::to_string(destVec.x) + ", " +
	// std::to_string(destVec.y));
	if (renderMissionRoute)
	{
		this->requestMissionPath(destVec);

		std::lock_guard<std::mutex> lock(pathMutex);
		this->renderPath(destVec, trace->m_nColour, trace->m_bFriendly, missionNodesCount, m_ResultNodes.data(),
						 missionDistance, m_LineVerts.data());
	}
}
