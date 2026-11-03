#pragma once

/*
	Plugin-SDK (Grand Theft Auto) source file
	Authors: GTA Community. See more here
	https://github.com/DK22Pac/plugin-sdk
	Do not delete this comment block. Respect others' work!
*/

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <future>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <string>

#include "CFont.h"
#include "CGeneral.h"
#include "CHudColours.h"
#include "CMenuManager.h"
#include "CObject.h"
#include "CPed.h"
#include "CPickup.h"
#include "CPickups.h"
#include "CPools.h"
#include "CRadar.h"
#include "CStreaming.h"
#include "CTheScripts.h"
#include "CVehicle.h"
#include "CWorld.h"
#include "Color.h"
#include "RenderWare.h"
#include "common.h"
#include "d3d9.h"
#include "plugin.h"

#include "util/Bools.h"
#include "util/Config.h"
#include "util/Logger.h"
#include "util/Misc.h"
#include "util/Render.h"

/*
	#define MAX_NODE_POINTS 50000
	#define GPS_LINE_WIDTH  4.0f
	#define GPS_LINE_R  180
	#define GPS_LINE_G  24
	#define GPS_LINE_B  24
	#define GPS_LINE_A  255
	#define MAX_TARGET_DISTANCE 10.0f
*/

#define MAX_NODE_POINTS 5000

// Anything outside this range (world units) can't be a real path node, so it's treated as corrupt data and skipped.
#define GPS_WORLD_LIMIT 6000.0f

// Two consecutive route nodes further apart than this (world units) are never joined by a line. This stops a single
// bad node from drawing a long streak across the radar.
#define GPS_MAX_SEGMENT_LENGTH 750.0f

// SA-MP only: how long to wait after samp.dll is detected before hooking the game events, so that our hooks
// are installed after SA-MP has installed its own (milliseconds).
#define GPS_SAMP_HOOK_DELAY_MS 5000

// Directory this ASI (not gta_sa.exe) lives in, with trailing slash. The ini and log files are kept here,
// independent of the current working directory.
inline std::string GpsModuleDir()
{
	HMODULE mod = nullptr;
	if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
							reinterpret_cast<LPCSTR>(&GpsModuleDir), &mod))
		return std::string();

	char buf[MAX_PATH] = {};
	const DWORD n = GetModuleFileNameA(mod, buf, MAX_PATH);
	if (n == 0 || n >= MAX_PATH)
		return std::string();

	std::string p(buf, n);
	const size_t slash = p.find_last_of("\\/");
	return slash == std::string::npos ? std::string() : p.substr(0, slash + 1);
}

class GPS
{
  private:
	void Run();
	void GameEventHandle();
	void DrawHudEventHandle();
	void DrawRadarOverlayHandle();
	void renderMissionTrace(tRadarTrace *trace);
	// Self explanatory.
	void calculatePath(const CVector &startPosn, const CVector &destPosn, bool respectLanes, bool waterNav,
					   short &nodesCount, CNodeAddress *resultNodes, float &gpsDistance);
	// Snapshots player/vehicle data on the game thread for the worker thread.
	bool captureSearchParams(CVector &startPosn, bool &respectLanes, bool &waterNav);
	// True when running inside SA-MP (samp.dll loaded).
	static bool IsSAMP();
	// Applies the path-node streaming patches (also re-applied later under SA-MP in case they get overwritten).
	void ApplyPatches();
	// SA-MP: asks the streaming system to load every path-node area (see GPS.cpp).
	void RequestAllPathAreas();
	// SA-MP: true when all 64 path-node areas are fully loaded.
	static bool PathAreasLoaded();
	// Registers the game/radar event callbacks.
	void RegisterEvents();
	// SA-MP: waits until SA-MP has installed its own hooks, then registers the callbacks.
	static DWORD WINAPI DeferredRegisterThread(LPVOID param);
	void requestTargetPath(CVector destPosn);
	void requestMissionPath(CVector destPosn);
	void renderPath(CVector tracePos, short color, bool friendly, short &nodesCount, CNodeAddress *resultNodes,
					float &gpsDistance, RwIm2DVertex *lineVerts);

	bool renderMissionRoute;
	bool renderTargetRoute;
	float targetDistance;
	short targetNodesCount;
	float angle;
	unsigned int vertIndex = 0;
	// These will be used for mission objectives
	float missionDistance;
	short missionNodesCount;
	util::Config cfg = util::Config((GpsModuleDir() + "SA.GPS.CONF.ini").c_str());
	util::Logger logger = util::Logger(false);
	CPed *player;
	tRadarTrace *mTrace;
	CPathNode *currentNode;
	CVector destVec;
	CVector targetTracePos;
	CVector2D targetScreen;
	CVector2D tmpPoint;
	CVector2D dir;
	CVector nodePosn;
	CVector2D shift[2];
	std::array<char, 1024> pathNodesToStream{};
	std::array<int, 50000> pathNodes{};
	std::array<CVector2D, MAX_NODE_POINTS> tmpNodePoints{};
	// Per node: the node could be read and has sane coordinates. Per segment i->i+1: both ends are valid and close.
	std::array<bool, MAX_NODE_POINTS> tmpNodeOk{};
	std::array<bool, MAX_NODE_POINTS> tmpSegmentOk{};
	std::array<CNodeAddress, MAX_NODE_POINTS> t_ResultNodes{};
	std::array<RwIm2DVertex, MAX_NODE_POINTS * 4> t_LineVerts{};
	std::array<CNodeAddress, MAX_NODE_POINTS> m_ResultNodes{};
	std::array<RwIm2DVertex, MAX_NODE_POINTS * 4> m_LineVerts{};
	std::future<void> targetFuture;
	std::future<void> missionFuture;
	std::mutex pathMutex;
	DWORD lastPathAreaRequest = 0;
	// SA-MP: becomes true once every path-node area has finished streaming in (stays true afterwards).
	bool pathAreasReady = false;
	// Bumped whenever the waypoint changes, so a search still running for the old waypoint can't publish its result.
	std::atomic<unsigned> targetGeneration{0};
	CVector lastRequestedDest;
	bool hasRequestedDest = false;

  public:
	inline GPS()
	{
		this->Run();
	}
} GPSLineRedux;