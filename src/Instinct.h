//  Instinct Reactive Planning Library
//  Copyright (c) 2016  Robert H. Wortham <r.h.wortham@gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

#ifndef _INSTINCT_H_
#define _INSTINCT_H_

// these are the 6 types of nodes
#define	INSTINCT_NODE_TYPES				6
#define	INSTINCT_ACTIONPATTERN			0
#define INSTINCT_ACTIONPATTERNELEMENT	1
#define INSTINCT_COMPETENCE				2
#define INSTINCT_COMPETENCEELEMENT		3
#define INSTINCT_DRIVE					4
#define INSTINCT_ACTION					5

// these are the valid comparators for senses
#define INSTINCT_COMPARATOR_EQ	0
#define INSTINCT_COMPARATOR_NE	1
#define INSTINCT_COMPARATOR_GT	2
#define INSTINCT_COMPARATOR_LT	3
#define INSTINCT_COMPARATOR_TR	4 // TR always returns true and does not bother to read the sensor. Use for default CE's.
#define INSTINCT_COMPARATOR_FL	5 // always returns false - mainly useful for debugging

// these are the 3 states a Drive can be in
#define INSTINCT_STATUS_NOTRUNNING	0
#define INSTINCT_STATUS_RUNNING		1
#define INSTINCT_STATUS_INTERRUPTED	2

// these are the valid return types from all node processing
// only success will increment the uiRuntime_SuccessCount
#define INSTINCT_FAIL			0
#define INSTINCT_SUCCESS		1
#define INSTINCT_IN_PROGRESS	2
#define INSTINCT_ERROR			3

// these values are used for the bRuntime_Status flag in ActionPatternElementType and CompetenceElementType
#define INSTINCT_RUNTIME_NOT_TESTED		0
#define	INSTINCT_RUNTIME_SUCCESS		1
#define INSTINCT_RUNTIME_IN_PROGRESS	2
#define INSTINCT_RUNTIME_ERROR			3
#define INSTINCT_RUNTIME_FAILED			4
#define INSTINCT_RUNTIME_NOT_RELEASED	5

// macros to split the return value into control and data
// higher bits can be used to return data
#define INSTINCT_RTN(rtn) ((rtn) & 0x03)
#define INSTINCT_RTN_DATA(rtn) ((rtn) >> 2)
#define INSTINCT_RTN_COMBINE(rtn, data) (((rtn) & 0x03) | ((data) << 2))

namespace Instinct {

// for Arduino, use single bytes for Node ID's and therefore node counters etc, otherwise use unsigned int
#ifndef _MSC_VER
	typedef unsigned char instinctID;
	typedef unsigned char senseID;
	typedef unsigned char actionID;
#else
	typedef unsigned int instinctID;
	typedef unsigned int senseID;
	typedef unsigned int actionID;
#endif

#define INSTINCT_MAX_INSTINCTID  ((0x01 << (sizeof(Instinct::instinctID)*8))-1)

typedef struct {
	senseID bSenseID;
	unsigned char bComparator;
	int nSenseValue;
	int nSenseHysteresis;
	int nSenseFlexLatchHysteresis;
	unsigned char bRuntime_Released;
} ReleaserType;

typedef struct {
	instinctID bPriority;
} PriorityType;

typedef struct {
	instinctID bPriority;
	instinctID bRampIncrement;
	instinctID bUrgencyMultiplier;
	instinctID bRuntime_Priority;
	unsigned char bRuntime_Checked;
	unsigned int uiRampInterval;
	unsigned int uiRuntime_RampIntervalCounter;
} DrivePriorityType;

typedef struct {
	unsigned int uiInterval;
	unsigned int uiRuntime_IntervalCounter;
} FrequencyType;

typedef struct {
	unsigned char bRetryLimit;
	unsigned char bRuntime_RetryCount;
} RetryType;

typedef struct {
	unsigned int uiRuntime_ExecutionCount;
	unsigned int uiRuntime_SuccessCount;
	unsigned char bMonitorFlags; // bit 0 = execution, bit 1 = success, bit 2 = pending, bit 3 = fail, bit 4 = error, bit 5 sense
} RuntimeCounters;

typedef struct {
	instinctID bRuntime_ElementID;
} RuntimeReferences;

typedef struct {
	instinctID bRuntime_ParentID;
	instinctID bRuntime_ChildID;
} ParentChildReferences;

typedef struct {
	instinctID bRuntime_CurrentElementID;
} ActionPatternType;

typedef struct {
	ParentChildReferences sParentChild;
	instinctID bOrder;
	unsigned char bRuntime_Status; // see INSTINCT_RUNTIME_*
} ActionPatternElementType;


typedef struct {
	instinctID bRuntime_CurrentElementID;
	unsigned char bUseORWithinCEGroup;
} CompetenceType;

typedef struct {
	ReleaserType sReleaser;
	RetryType sRetry;
	PriorityType sPriority;
	ParentChildReferences sParentChild;
	unsigned char bRuntime_Status; // see INSTINCT_RUNTIME_*
} CompetenceElementType;

typedef struct {
	// CompetenceType sCompetence;
	ReleaserType sReleaser;
	DrivePriorityType sDrivePriority;
	FrequencyType sFrequency;
	instinctID bRuntime_ChildID;
	unsigned char bRuntime_Status; // see INSTINCT_STATUS_*
} DriveType;

typedef struct {
	actionID bActionID;
	int nActionValue;
	unsigned char bRuntime_CheckForComplete;
} ActionType;

typedef struct {
	RuntimeReferences sReferences;
	RuntimeCounters sCounters;
	union {
		ActionPatternType sActionPattern;
		ActionPatternElementType sActionPatternElement;
		CompetenceType sCompetence;
		CompetenceElementType sCompetenceElement;
		DriveType sDrive;
		ActionType sAction;
	};
} PlanElement;

typedef struct {
	unsigned char bNodeType;
	PlanElement sElement;
} PlanNode;


class Senses {
public:
	virtual int readSense(const senseID nSense) = 0;
};

class Actions {
public:
	virtual unsigned char executeAction(const actionID nAction, const int nActionValue, const unsigned char bCheckForComplete) = 0;
};

class Monitor {
public:
	virtual unsigned char nodeExecuted(const PlanNode * pPlanNode) = 0;
	virtual unsigned char nodeSuccess(const PlanNode * pPlanNode) = 0;
	virtual unsigned char nodeInProgress(const PlanNode * pPlanNode) = 0;
	virtual unsigned char nodeFail(const PlanNode * pPlanNode) = 0;
	virtual unsigned char nodeError(const PlanNode * pPlanNode) = 0;
	virtual unsigned char nodeSense(const ReleaserType *pReleaser, const int nSenseValue) = 0;
};

class PlanManager {
public:
	PlanManager(instinctID *pPlanSize, Senses *pSenses, Actions *pActions, Monitor *pMonitor);
	void setPlanID(const int nPlanID);
	int getPlanID(void);
	unsigned char executeCommand(const char * pCmd, char *pRtnBuff, const int nRtnBuffLen);
	unsigned char initialisePlan(instinctID *pPlanSize); //reset the current plan
	instinctID planSize(const unsigned char nNodeType);
	instinctID planSize(void);
	void planSize(instinctID *pPlanSize);
	unsigned int planUsage(instinctID *pPlanSize);
	instinctID maxElementID(void);
	unsigned char addNode(PlanNode *pPlanNode); // add a plan node to the plan
	unsigned char addDrive(const instinctID bRuntime_ElementID, const instinctID bRuntime_ChildID, const instinctID bPriority, const unsigned int uiInterval,
		const senseID bSenseID, const unsigned char bComparator, const int nSenseValue, const int nSenseHysteresis, const int nSenseFlexLatchHysteresis,
		const instinctID bRampIncrement, const instinctID bUrgencyMultiplier, const instinctID uiRampInterval);
	unsigned char addCompetenceElement(const instinctID bRuntime_ElementID, const instinctID bRuntime_ParentID, const instinctID bRuntime_ChildID, const instinctID bPriority, unsigned char bRetryLimit,
		const senseID bSenseID, const unsigned char bComparator, const int nSenseValue,
		const int nSenseHysteresis, const int nSenseFlexLatchHysteresis);
	unsigned char addActionPattern(const instinctID bRuntime_ElementID);
	unsigned char addActionPatternElement(const instinctID bRuntime_ElementID, const instinctID bRuntime_ParentID, const instinctID bRuntime_ChildID, const instinctID bOrder);
	unsigned char addAction(const instinctID bRuntime_ElementID, const actionID bActionID, const int nActionValue);
	unsigned char addCompetence(const instinctID bRuntime_ElementID, const unsigned char bUseORWithinCEGroup);
	unsigned char getNode(PlanNode *pPlanNode, const instinctID nElementID); // fill pointer to a plan node based on ElementID
	unsigned char updateNode(PlanNode *pPlanNode); // update a plan node based on ElementID and node type
	unsigned char monitorNode(const instinctID bRuntime_ElementID, const unsigned char bMonitorExecuted, const unsigned char bMonitorSuccess,
		const unsigned char bMonitorPending, const unsigned char bMonitorFail, const unsigned char bMonitorError, const unsigned char bMonitorSense);
	void setGlobalMonitorFlags(const unsigned char bMonitorExecuted, const unsigned char bMonitorSuccess,
		const unsigned char bMonitorPending, const unsigned char bMonitorFail, const unsigned char bMonitorError, const unsigned char bMonitorSense);
	int sizeFromNodeType(const unsigned char nNodeType);
	unsigned char setDrivePriority(const instinctID bRuntime_ElementID, const instinctID bPriority);
	unsigned char setRuntimeDrivePriority(const instinctID bRuntime_ElementID, const instinctID bPriority);
	instinctID getDrivePriority(const instinctID bRuntime_ElementID);
	instinctID getRuntimeDrivePriority(const instinctID bRuntime_ElementID);


	protected:
	Senses * _pSenses;
	Actions * _pActions;
	Monitor * _pMonitor;
	instinctID _nPlanSize[INSTINCT_NODE_TYPES];
	PlanElement * _pPlan[INSTINCT_NODE_TYPES];
	PlanElement * _pLastNode[INSTINCT_NODE_TYPES];
	instinctID _nNodeCount[INSTINCT_NODE_TYPES];
	unsigned char _bGlobalMonitorFlags;
	int _nPlanID; // a numeric identifier for the plan, useful where there are many plans

	PlanElement * findElement(const instinctID bElementID);
	PlanElement * findElementAndType(const instinctID bElementID, unsigned char *pNodeType);
	PlanElement * findElement(const instinctID bElementID, const unsigned char nNodeType);
	PlanElement * findChildAorAPorC(const instinctID bElementID, unsigned char *pNodeType);
	void countExecution(PlanElement *pElement, const unsigned char nNodeType);
	void countSuccess(PlanElement *pElement, const unsigned char nNodeType);
	void countInProgress(PlanElement *pElement, const unsigned char nNodeType);
	void countFail(PlanElement *pElement, const unsigned char nNodeType);
	void countError(PlanElement *pElement, const unsigned char nNodeType);
	void countSense(PlanElement *pElement, ReleaserType *pReleaser, const int nSenseValue);
};


class Planner : public PlanManager {
public:
	Planner(instinctID *pPlanSize, Senses *pSenses, Actions *pActions, Monitor *pMonitor);
	unsigned char runPlan(void);
	unsigned char processTimers(const unsigned int uiTime);

	int readSense(const senseID nSense);
	unsigned char executeAction(const actionID nAction, const int nActionValue, const unsigned char bCheckForComplete);

private:
	unsigned char executeDrive(PlanElement * pDrive);
	unsigned char executeCE(PlanElement *pCompetenceElement, PlanElement *pDrive);
	unsigned char executeAction(PlanElement *pAction, PlanElement *pDrive);
	unsigned char executeActionPattern(PlanElement *pActionPattern, PlanElement *pDrive);
	unsigned char executeCompetence(PlanElement *pCompetence, PlanElement *pDrive);

	// these are the second level functions for complex logic operations
	unsigned char executeCompetenceInitial(PlanElement *pCompetence, PlanElement *pDrive);
	unsigned char executeCompetenceSubsequent(PlanElement *pCompetence, PlanElement *pDrive);
	unsigned char processExecutedCE(PlanElement *pCE, PlanElement *pCompetence, PlanElement *pDrive, const unsigned char bRetVal);
	unsigned char clearCENotReleasedStatus(const instinctID bParentCompetenceID, const instinctID nCEPriority);

	// these are essentially helper functions for the main private functions above
	unsigned char checkReleaser(PlanElement *pPlanElement, ReleaserType * pReleaser, DriveType *pDrive);
	unsigned char checkDriveFrequency(DriveType *pDrive);
	PlanElement * findCEForReleaserCheck(const instinctID bParentCompetenceID, const instinctID bLastElementPriority);
	PlanElement * findNextCE(const instinctID bParentCompetenceID, const instinctID bLastElementPriority,
		const unsigned char bNextLevel, const unsigned char bIncludeNotReleased);
	PlanElement * findNextAPE(const instinctID bParentActionPatternID, const instinctID bLastElementOrder);
	unsigned char executeAPE(PlanElement *pActionPatternElement, PlanElement *pDrive);
	unsigned char testCEForRunningAP(PlanElement *pCE);
	unsigned char clearCECompletedFlags(instinctID bParentID);
	unsigned char clearAPECompletedFlags(instinctID bParentID);
};

class CmdPlanner : public Planner {
public:
	CmdPlanner(instinctID *pPlanSize, Senses *pSenses, Actions *pActions, Monitor *pMonitor);
	const char * help(void);
	unsigned char executeCommand(const char * pCmd, char *pRtnBuff, const int nRtnBuffLen); // execute a plan command and return data as a string
	unsigned char displayNode(char *pStrBuff, const int nBuffLen, const instinctID nElementID); // fill a string buffer with the command needed to add the node
	unsigned char displayNode(char *pStrBuff, const int nBuffLen, const PlanNode *pPlanNode);
	unsigned char displayNodeCounters(char *pStrBuff, const int nBuffLen, const instinctID nElementID); // fill a string buffer with the counters for the node
	unsigned char displayNodeCounters(char *pStrBuff, const int nBuffLen, const PlanNode *pPlanNode);
	unsigned char displayReleaser(char *pStrBuff, const int nBuffLen, const ReleaserType *pReleaser);
};


// ** the following structures and class are separate to the Planner itself, but allow names and ID's to be stored and retrieved
// ** this is useful for debugging or for textual monitoring of plan operation

// structures to store a variable number of variable length names associated with IDs.
// this struct holds an ID and a variable length zero terminated name
typedef struct {
	instinctID bRuntime_ElementID;
	char szName[2]; // names are variable length and zero terminated
} ElementNameEntryType;

// this struct stores the total buffer size, the number of entries it contains and the storage for the entries
typedef struct {
	unsigned int uiBuffLen;
	instinctID bEntryCount;
	ElementNameEntryType sEntry[2]; // there will be uiEntryCount of these
} ElementNameBufferType;

class Names {
public:
	Names(const unsigned int uiBufferSize);
	unsigned char addElementName(const instinctID bRuntime_ElementID, char *pElementName);
	char * getElementName(const instinctID bRuntime_ElementID);
	instinctID getElementID(const char *pName);
	unsigned char clearElementNames(void);
	instinctID elementNameCount(void);
	unsigned char * elementBuffer(void);
	unsigned int elementBufferSize(void);
	instinctID maxElementNameID(void);

private:
	// pointer to the buffer containing all the names
	ElementNameBufferType *pElementNameBuffer;
};

} // /namespace Instinct

#endif // _INSTINCT_H_
