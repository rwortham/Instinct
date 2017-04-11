//  Memory Manager for Instinct Plans
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

#include <stdafx.h>

#ifndef _MSC_VER
	#include "Arduino.h"
#endif

#include "Instinct.h"

namespace Instinct {

PlanManager::PlanManager(instinctID *pPlanSize, Senses *pSenses, Actions *pActions, Monitor *pMonitor)
{
	_nPlanID = 0;
	_pSenses = pSenses;
	_pActions = pActions;
	_pMonitor = pMonitor;

	for (unsigned char i = 0; i < INSTINCT_NODE_TYPES; i++)
	{
		_pPlan[i] = 0;
		_pLastNode[i] = 0;
		_nPlanSize[i] = 0;
		_nNodeCount[i] = 0;
	}

	initialisePlan(pPlanSize);
}

// the PlanID is a useful identifier to identify which plan we are using, but it need not be used
void PlanManager::setPlanID(const int nPlanID)
{
	_nPlanID = nPlanID;
}

int PlanManager::getPlanID(void)
{
	return _nPlanID;
}


// Add a Drive to the plan
unsigned char PlanManager::addDrive(const instinctID bRuntime_ElementID, const instinctID bRuntime_ChildID, const instinctID bPriority,
	const unsigned int uiInterval, const senseID bSenseID, const unsigned char bComparator, const int nSenseValue,
	const int nSenseHysteresis, const int nSenseFlexLatchHysteresis,
	const instinctID bRampIncrement, const instinctID bUrgencyMultiplier, const instinctID uiRampInterval)
{
	Instinct::PlanNode planNode;

	memset(&planNode, 0, sizeof(planNode));

	planNode.bNodeType = INSTINCT_DRIVE;
	planNode.sElement.sReferences.bRuntime_ElementID = bRuntime_ElementID;
	planNode.sElement.sDrive.bRuntime_ChildID = bRuntime_ChildID;
	planNode.sElement.sDrive.sReleaser.bSenseID = bSenseID;
	planNode.sElement.sDrive.sReleaser.bComparator = bComparator;
	planNode.sElement.sDrive.sReleaser.nSenseValue = nSenseValue;
	planNode.sElement.sDrive.sReleaser.nSenseHysteresis = nSenseHysteresis;
	planNode.sElement.sDrive.sReleaser.nSenseFlexLatchHysteresis = nSenseFlexLatchHysteresis;
	planNode.sElement.sDrive.sDrivePriority.bPriority = bPriority;
	planNode.sElement.sDrive.sDrivePriority.bRampIncrement = bRampIncrement;
	planNode.sElement.sDrive.sDrivePriority.bUrgencyMultiplier = bUrgencyMultiplier;
	planNode.sElement.sDrive.sDrivePriority.uiRampInterval = uiRampInterval;
	planNode.sElement.sDrive.sFrequency.uiInterval = uiInterval;

	return addNode(&planNode);
}

unsigned char PlanManager::addCompetenceElement(const instinctID bRuntime_ElementID, const instinctID bRuntime_ParentID,
												const instinctID bRuntime_ChildID, const instinctID bPriority, unsigned char bRetryLimit,
	const senseID bSenseID, const unsigned char bComparator, const int nSenseValue,
	const int nSenseHysteresis, const int nSenseFlexLatchHysteresis)
{
	Instinct::PlanNode planNode;

	memset(&planNode, 0, sizeof(planNode));

	planNode.bNodeType = INSTINCT_COMPETENCEELEMENT;
	planNode.sElement.sReferences.bRuntime_ElementID = bRuntime_ElementID;
	planNode.sElement.sCompetenceElement.sParentChild.bRuntime_ParentID = bRuntime_ParentID;
	planNode.sElement.sCompetenceElement.sParentChild.bRuntime_ChildID = bRuntime_ChildID;
	planNode.sElement.sCompetenceElement.sReleaser.bSenseID = bSenseID;
	planNode.sElement.sCompetenceElement.sReleaser.bComparator = bComparator;
	planNode.sElement.sCompetenceElement.sReleaser.nSenseValue = nSenseValue;
	planNode.sElement.sCompetenceElement.sReleaser.nSenseHysteresis = nSenseHysteresis;
	planNode.sElement.sCompetenceElement.sReleaser.nSenseFlexLatchHysteresis = nSenseFlexLatchHysteresis;
	planNode.sElement.sCompetenceElement.sPriority.bPriority = bPriority;
	planNode.sElement.sCompetenceElement.sRetry.bRetryLimit = bRetryLimit;

	return addNode(&planNode);
}

unsigned char PlanManager::addActionPattern(const instinctID bRuntime_ElementID)
{
	Instinct::PlanNode planNode;

	memset(&planNode, 0, sizeof(planNode));

	planNode.bNodeType = INSTINCT_ACTIONPATTERN;
	planNode.sElement.sReferences.bRuntime_ElementID = bRuntime_ElementID;

	return addNode(&planNode);
}

unsigned char PlanManager::addActionPatternElement(const instinctID bRuntime_ElementID, const instinctID bRuntime_ParentID,
													const instinctID bRuntime_ChildID, const instinctID bOrder)
{
	Instinct::PlanNode planNode;

	memset(&planNode, 0, sizeof(planNode));

	planNode.bNodeType = INSTINCT_ACTIONPATTERNELEMENT;
	planNode.sElement.sReferences.bRuntime_ElementID = bRuntime_ElementID;
	planNode.sElement.sActionPatternElement.sParentChild.bRuntime_ParentID = bRuntime_ParentID;
	planNode.sElement.sActionPatternElement.sParentChild.bRuntime_ChildID = bRuntime_ChildID;
	planNode.sElement.sActionPatternElement.bOrder = bOrder;

	return addNode(&planNode);
}

unsigned char PlanManager::addAction(const instinctID bRuntime_ElementID, const actionID bActionID, const int nActionValue)
{
	Instinct::PlanNode planNode;

	memset(&planNode, 0, sizeof(planNode));

	planNode.bNodeType = INSTINCT_ACTION;
	planNode.sElement.sReferences.bRuntime_ElementID = bRuntime_ElementID;
	planNode.sElement.sAction.bActionID = bActionID;
	planNode.sElement.sAction.nActionValue = nActionValue;

	return addNode(&planNode);
}

unsigned char PlanManager::addCompetence(const instinctID bRuntime_ElementID, const unsigned char bUseORWithinCEGroup)
{
	Instinct::PlanNode planNode;

	memset(&planNode, 0, sizeof(planNode));

	planNode.bNodeType = INSTINCT_COMPETENCE;
	planNode.sElement.sReferences.bRuntime_ElementID = bRuntime_ElementID;
	planNode.sElement.sCompetence.bUseORWithinCEGroup = bUseORWithinCEGroup;

	return addNode(&planNode);
}


// return the number of elements with a given node type
instinctID PlanManager::planSize(const unsigned char nNodeType)
{
	return _nNodeCount[nNodeType];
}

// return the total number of elements in the plan
instinctID PlanManager::planSize(void)
{
	unsigned int nSize = 0;

	for (unsigned char i = 0; i < INSTINCT_NODE_TYPES; i++)
	{
		nSize += _nNodeCount[i];
	}

	return nSize;
}

// copy element counts to buffer
void PlanManager::planSize(instinctID *pPlanSize)
{
	if (!pPlanSize)
		return;

	for (unsigned char i = 0; i < INSTINCT_NODE_TYPES; i++)
	{
		*(pPlanSize+i) = _nNodeCount[i];
	}
}

// return the calculated memory usage of a plan with NodeCount[] elements
// if pNodeCount is null, then return current memory usage
unsigned int PlanManager::planUsage(instinctID *pNodeCount)
{
	unsigned int nSize = 0;

	if (!pNodeCount)
		pNodeCount = _nNodeCount;

	for (unsigned char i = 0; i < INSTINCT_NODE_TYPES; i++)
	{
		nSize += pNodeCount[i] * sizeFromNodeType(i);
	}
	return nSize;
}

// Run over all the plan buffers and find the highest ElementID
// there may be gaps but we can then assume all plan elements have an ID between 1 and this ElementID
instinctID PlanManager::maxElementID(void)
{
	instinctID bHighest = 0;
	PlanElement *pElement;
	int nSize;

	for (unsigned char i = 0; i < INSTINCT_NODE_TYPES; i++)
	{
		pElement = _pPlan[i];
		nSize = sizeFromNodeType(i);
		for (instinctID j = 0; j < _nNodeCount[i]; j++)
		{
			if (bHighest < pElement->sReferences.bRuntime_ElementID)
				bHighest = pElement->sReferences.bRuntime_ElementID;
			pElement = (PlanElement *)((unsigned char *)pElement + nSize);
		}
	}
	return bHighest;
}


//reset the current plan
unsigned char PlanManager::initialisePlan(instinctID *pPlanSize)
{
	// set up the memory buffer for the various node types
	for (unsigned char i = 0; i < INSTINCT_NODE_TYPES; i++)
	{
		if (_pPlan[i])
		{
			free((void *)_pPlan[i]);
			_pPlan[i] = 0;
			_pLastNode[i] = 0;
			_nPlanSize[i] = 0;
			_nNodeCount[i] = 0;
		}
		unsigned int nSize = *(pPlanSize + i);
		unsigned int nBuffSize;
		if (nSize != 0)
		{
			nBuffSize = nSize * sizeFromNodeType(i);
			_pPlan[i] = (PlanElement *)malloc(nBuffSize);

			// if the malloc fails this is a fatal error as insufficient room for plan
			if (!_pPlan[i])
				return false;
			memset(_pPlan[i], 0, nBuffSize);
			_nPlanSize[i] = nSize;
			_pLastNode[i] = _pPlan[i];
		}
	}
	// all memory allocations successful
	return true;
}

// add a plan node to the end of the plan
// check there is space in the correct plan buffer first
// update _nNodeCount and _pLastNode
unsigned char PlanManager::addNode(PlanNode *pNode)
{
	int nNodeSize;
	int nLastNodeSize;
	unsigned char nNodeType;

	if (!pNode)
		return false;
	nNodeType = pNode->bNodeType;
	nNodeSize = sizeFromNodeType(nNodeType);
	if (!nNodeSize)
		return false; // not a valid node type

	if (!_pLastNode[nNodeType])
		return false; // no buffer for nodes of this type

	// check there is room to add the new plan node
	if (_nNodeCount[nNodeType] >= _nPlanSize[nNodeType])
		return false;

	// Get size of last node, or zero if no nodes yet
	nLastNodeSize = (_nNodeCount[nNodeType] ? nNodeSize : 0);

	// do any initial setup of Runtime data for particular node types here
	// assume all Runtime values initially come in with zero values
	switch (nNodeType)
	{
	case INSTINCT_DRIVE:
		// for Drive we need to copy the initial Drive Priority to the Runtime value before any processing starts
		pNode->sElement.sDrive.sDrivePriority.bRuntime_Priority = pNode->sElement.sDrive.sDrivePriority.bPriority;
		break;
	}

	// all is good, so add the node
	_pLastNode[nNodeType] = (PlanElement *)((unsigned char *)_pLastNode[nNodeType] + nLastNodeSize);
	memcpy(_pLastNode[nNodeType], &(pNode->sElement), nNodeSize);
	_nNodeCount[nNodeType]++;

	return true;
}

// copy a plan node into the supplied buffer, based on its elementID
unsigned char PlanManager::getNode(PlanNode *pPlanNode, const instinctID uiElementID)
{
	PlanElement * pPlanElement;

	if (!pPlanNode)
		return false;

	// run over all the plan buffers, searching for the correct ElementID in each buffer
	// this could be quite slow
	for (unsigned char nNodeType = 0; nNodeType < INSTINCT_NODE_TYPES; nNodeType++)
	{
		pPlanElement = findElement(uiElementID, nNodeType);

		if (pPlanElement)
		{
			pPlanNode->bNodeType = nNodeType;
			memcpy(&(pPlanNode->sElement), pPlanElement, sizeFromNodeType(nNodeType));

			return true; // all done
		}
	}

	return false; // no matching node found
}

// update a plan node based on its node type and elementID
// first find a matching node, then copy its values over
unsigned char  PlanManager::updateNode(PlanNode *pNode)
{
	PlanElement * pPlanElement;

	// check the supplied node for validity
	if (!pNode)
		return false;

	// search over all nodes of the matching node type
	pPlanElement = findElement(pNode->sElement.sReferences.bRuntime_ElementID, pNode->bNodeType);
	if (!pPlanElement) // not found
		return false;

	memcpy(pPlanElement, &(pNode->sElement), sizeFromNodeType(pNode->bNodeType));

	return true; // all done
}

// set the monitoring flags on a node
unsigned char PlanManager::monitorNode(const instinctID bRuntime_ElementID, const unsigned char bMonitorExecuted, const unsigned char bMonitorSuccess,
	const unsigned char bMonitorPending, const unsigned char bMonitorFail, const unsigned char bMonitorError, const unsigned char bMonitorSense)
{
	PlanElement *pElement;

	if (!_pMonitor)
		return false;
	pElement = findElement(bRuntime_ElementID);
	if (!pElement)
		return false;
	pElement->sCounters.bMonitorFlags = (bMonitorExecuted ? 0x01 : 0x0) | (bMonitorSuccess ? 0x02 : 0x0) | (bMonitorPending ? 0x04 : 0x0) |
		(bMonitorFail ? 0x08 : 0x0) | (bMonitorError ? 0x10 : 0x0) | (bMonitorSense ? 0x20 : 0x0);
	return true;
}

// set the global flags that will cause monitoring of all executions and successes
void PlanManager::setGlobalMonitorFlags(const unsigned char bMonitorExecuted, const unsigned char bMonitorSuccess,
	const unsigned char bMonitorPending, const unsigned char bMonitorFail, const unsigned char bMonitorError, const unsigned char bMonitorSense)
{
	_bGlobalMonitorFlags = (bMonitorExecuted ? 0x01 : 0x0) | (bMonitorSuccess ? 0x02 : 0x0) | (bMonitorPending ? 0x04 : 0x0) |
		(bMonitorFail ? 0x08 : 0x0) | (bMonitorError ? 0x10 : 0x0) | (bMonitorSense ? 0x20 : 0x0);
}

// set the Drive Priority for the given element ID. Return 0 if not found
unsigned char PlanManager::setDrivePriority(const instinctID bRuntime_ElementID, const instinctID bPriority)
{
	PlanElement *pDrive = findElement(bRuntime_ElementID, INSTINCT_DRIVE);
	if (!pDrive)
		return false;

	pDrive->sDrive.sDrivePriority.bPriority = bPriority;

	return true;
}

// set the Drive Priority for the given element ID. Return 0 if not found
unsigned char PlanManager::setRuntimeDrivePriority(const instinctID bRuntime_ElementID, const instinctID bPriority)
{
	PlanElement *pDrive = findElement(bRuntime_ElementID, INSTINCT_DRIVE);
	if (!pDrive)
		return false;

	pDrive->sDrive.sDrivePriority.bRuntime_Priority = bPriority;

	return true;
}

// get the Drive Priority for the given element ID. Return 0 if not found
instinctID PlanManager::getDrivePriority(const instinctID bRuntime_ElementID)
{
	PlanElement *pDrive = findElement(bRuntime_ElementID, INSTINCT_DRIVE);
	if (!pDrive)
		return 0;

	return pDrive->sDrive.sDrivePriority.bPriority;
}

// get the Runtime Drive Priority for the given element ID. Return 0 if not found
instinctID PlanManager::getRuntimeDrivePriority(const instinctID bRuntime_ElementID)
{
	PlanElement *pDrive = findElement(bRuntime_ElementID, INSTINCT_DRIVE);
	if (!pDrive)
		return false;

	return pDrive->sDrive.sDrivePriority.bRuntime_Priority;
}

// find an element based on the supplied ElementID and NodeType
// return null pointer if no match
PlanElement * PlanManager::findElement(const instinctID bElementID, const unsigned char nNodeType)
{
	PlanElement *pPlanElement = 0;
	int nSize;

	nSize = sizeFromNodeType(nNodeType);
	if (!nSize) // invalid node type
		return 0;

	pPlanElement = _pPlan[nNodeType];

	for (instinctID i = 0; i < _nNodeCount[nNodeType]; i++)
	{
		if (pPlanElement->sReferences.bRuntime_ElementID == bElementID)
		{
			return pPlanElement;
		}
		pPlanElement = (PlanElement *)((unsigned char *)pPlanElement + nSize);
	}

	return 0; // no match
}

// find an element based on the supplied ElementID
// return null pointer if no match
PlanElement * PlanManager::findElement(const instinctID bElementID)
{
	PlanElement *pElement;
	for (int i = 0; i < INSTINCT_NODE_TYPES; i++)
	{
		pElement = findElement(bElementID, i);
		if (pElement)
		{
			return pElement;
		}
	}
	return 0;
}

// find an element based on the supplied ElementID, populate the NodeType element
// return null pointer if no match
PlanElement * PlanManager::findElementAndType(const instinctID bElementID, unsigned char *pNodeType)
{
	PlanElement *pElement;
	for (int i = 0; i < INSTINCT_NODE_TYPES; i++)
	{
		pElement = findElement(bElementID, i);
		if (pElement)
		{
			*pNodeType = i;
			return pElement;
		}
	}
	return 0;
}

// find an Action (A), an Action Pattern (AP), or a Competence (C)
// return pointer to the Element, and populate the NodeType
PlanElement * PlanManager::findChildAorAPorC(const instinctID bElementID, unsigned char *pNodeType)
{
	PlanElement *pPlanElement;

	if (pPlanElement = findElement(bElementID, INSTINCT_ACTION))
	{
		*pNodeType = INSTINCT_ACTION;
		return pPlanElement;
	}
	else if (pPlanElement = findElement(bElementID, INSTINCT_ACTIONPATTERN))
	{
		*pNodeType = INSTINCT_ACTIONPATTERN;
		return pPlanElement;
	}
	else if (pPlanElement = findElement(bElementID, INSTINCT_COMPETENCE))
	{
		*pNodeType = INSTINCT_COMPETENCE;
		return pPlanElement;
	} else
	{
		*pNodeType = INSTINCT_NODE_TYPES; // this will always be an invalid node type (Defensive Coding!)
	}
	return 0;
}

// Count runtime execution and notify Monitor if enabled
void PlanManager::countExecution(PlanElement *pElement, const unsigned char bNodeType)
{
	pElement->sCounters.uiRuntime_ExecutionCount++;
	if (_pMonitor && ((_bGlobalMonitorFlags & 0x01) || (pElement->sCounters.bMonitorFlags & 0x01)))
	{
		PlanNode sNode;
		int nNodeSize;
		nNodeSize = sizeFromNodeType(bNodeType);
		if (nNodeSize)
		{
			sNode.bNodeType = bNodeType;
			memcpy(&sNode.sElement, pElement, nNodeSize);
			_pMonitor->nodeExecuted(&sNode);
		}
	}
}

// Count runtime success and notify Monitor if enabled
void PlanManager::countSuccess(PlanElement *pElement, const unsigned char bNodeType)
{
	pElement->sCounters.uiRuntime_SuccessCount++;
	if (_pMonitor && ((_bGlobalMonitorFlags & 0x02) || (pElement->sCounters.bMonitorFlags & 0x02)))
	{
		PlanNode sNode;
		int nNodeSize;
		nNodeSize = sizeFromNodeType(bNodeType);
		if (nNodeSize)
		{
			sNode.bNodeType = bNodeType;
			memcpy(&sNode.sElement, pElement, nNodeSize);
			_pMonitor->nodeSuccess(&sNode);
		}
	}
}


// Count runtime pending and notify Monitor if enabled
void PlanManager::countInProgress(PlanElement *pElement, const unsigned char bNodeType)
{
	// pElement->sCounters.uiRuntime_SuccessCount++; // currently we are not stored values for pending
	if (_pMonitor && ((_bGlobalMonitorFlags & 0x04) || (pElement->sCounters.bMonitorFlags & 0x04)))
	{
		PlanNode sNode;
		int nNodeSize;
		nNodeSize = sizeFromNodeType(bNodeType);
		if (nNodeSize)
		{
			sNode.bNodeType = bNodeType;
			memcpy(&sNode.sElement, pElement, nNodeSize);
			_pMonitor->nodeInProgress(&sNode);
		}
	}
}


// Count runtime fail and notify Monitor if enabled
void PlanManager::countFail(PlanElement *pElement, const unsigned char bNodeType)
{
	// pElement->sCounters.uiRuntime_SuccessCount++; // currently we are not stored values for fail
	if (_pMonitor && ((_bGlobalMonitorFlags & 0x08) || (pElement->sCounters.bMonitorFlags & 0x08)))
	{
		PlanNode sNode;
		int nNodeSize;
		nNodeSize = sizeFromNodeType(bNodeType);
		if (nNodeSize)
		{
			sNode.bNodeType = bNodeType;
			memcpy(&sNode.sElement, pElement, nNodeSize);
			_pMonitor->nodeFail(&sNode);
		}
	}
}


// Count runtime error and notify Monitor if enabled
void PlanManager::countError(PlanElement *pElement, const unsigned char bNodeType)
{
	// pElement->sCounters.uiRuntime_SuccessCount++; // currently we are not stored values for error
	if (_pMonitor && ((_bGlobalMonitorFlags & 0x10) || (pElement->sCounters.bMonitorFlags & 0x10)))
	{
		PlanNode sNode;
		int nNodeSize;
		nNodeSize = sizeFromNodeType(bNodeType);
		if (nNodeSize)
		{
			sNode.bNodeType = bNodeType;
			memcpy(&sNode.sElement, pElement, nNodeSize);
			_pMonitor->nodeError(&sNode);
		}
	}
}


// Count runtime error and notify Monitor if enabled
void PlanManager::countSense(PlanElement *pElement, ReleaserType *pReleaser, const int nSenseValue)
{
	// pElement->sCounters.uiRuntime_SuccessCount++; // currently we are not stored values for sense
	if (_pMonitor && ((_bGlobalMonitorFlags & 0x20) || (pElement->sCounters.bMonitorFlags & 0x20)))
	{
			_pMonitor->nodeSense(pReleaser, nSenseValue);
	}
}


// returns the memory allocation needed for a node of a given type
// returns zero on error
int PlanManager::sizeFromNodeType(const unsigned char bNodeType)
{
	int nSize;

	switch (bNodeType)
	{
	case INSTINCT_ACTION:
		nSize = sizeof(ActionType);
		break;
	case INSTINCT_ACTIONPATTERNELEMENT:
		nSize = sizeof(ActionPatternElementType);
		break;
	case INSTINCT_ACTIONPATTERN:
		nSize = sizeof(ActionPatternType);
		break;
	case INSTINCT_COMPETENCEELEMENT:
		nSize = sizeof(CompetenceElementType);
		break;
	case INSTINCT_COMPETENCE:
		nSize = sizeof(CompetenceType);
		break;
	case INSTINCT_DRIVE:
		nSize = sizeof(DriveType);
		break;

	default:
		return 0;
	}

	return (nSize + sizeof(RuntimeReferences) + sizeof(RuntimeCounters));
}

} // /namespace Instinct