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

#include <stdafx.h>

#ifndef _MSC_VER
	#include "Arduino.h"
#endif

#include "Instinct.h"

namespace Instinct {

Planner::Planner(instinctID *pPlanSize, Senses *pSenses, Actions *pActions, Monitor *pMonitor)
	: PlanManager(pPlanSize, pSenses, pActions, pMonitor)
{
}

// Called by the robot using the Planner to decrement timers by the given amount.
// Expired timers are set to zero. Currently the only timers are those associated with Drives
// Timers are reset only when expired and tested in checkDriveFreqency()

// RAMP Processing: At each interval, the Runtime_Priority is increased by the Ramp Increment. In addition,
// once the drive’s sense allows it to be released, the priority is multiplied each ramp interval by the Urgency Multiplier.
// This mimics an exponential increase in drive priority, once the threshold has been reached.
// Since in the physical implementation the Ramp Multiplier will be a byte(0 - 255), the multiplier value is scaled back by a factor of 32,
// giving actual factors of 0 to 7, with a resolution of ~0.03.
unsigned char Planner::processTimers(const unsigned int uiTime)
{
	PlanElement *pPlanElement;
	int nSize;

	nSize = sizeFromNodeType(INSTINCT_DRIVE);
	pPlanElement = _pPlan[INSTINCT_DRIVE];

	if (!nSize || !pPlanElement) // should never happen
		return INSTINCT_ERROR;

	for (instinctID i = 0; i < _nNodeCount[INSTINCT_DRIVE]; i++)
	{
		// Frequency determines how often a Drive is processed
		// 0 implies Drive is processed on every cycle
		if (pPlanElement->sDrive.sFrequency.uiInterval) // only bother with this if Frequency Interval is set up
		{
			if (pPlanElement->sDrive.sFrequency.uiRuntime_IntervalCounter > uiTime)
				pPlanElement->sDrive.sFrequency.uiRuntime_IntervalCounter -= uiTime;
			else
				pPlanElement->sDrive.sFrequency.uiRuntime_IntervalCounter = 0;
		}

		// Ramp Interval determines how often the RAMP logic is run to alter Drive priority
		// 0 implies no Ramping of Drive priority
		if (pPlanElement->sDrive.sDrivePriority.uiRampInterval) // only bother with this if Ramp Interval is set up
		{
			if (pPlanElement->sDrive.sDrivePriority.uiRuntime_RampIntervalCounter > uiTime)
				pPlanElement->sDrive.sDrivePriority.uiRuntime_RampIntervalCounter -= uiTime;
			else
				pPlanElement->sDrive.sDrivePriority.uiRuntime_RampIntervalCounter = 0;

			// can can do the RAMP processing here to modify the Drive priority
			if (!pPlanElement->sDrive.sDrivePriority.uiRuntime_RampIntervalCounter)
			{
				pPlanElement->sDrive.sDrivePriority.uiRuntime_RampIntervalCounter = pPlanElement->sDrive.sDrivePriority.uiRampInterval;

				// avoid rollover
				if ((instinctID)-1 - pPlanElement->sDrive.sDrivePriority.bRuntime_Priority > pPlanElement->sDrive.sDrivePriority.bRampIncrement)
					pPlanElement->sDrive.sDrivePriority.bRuntime_Priority += pPlanElement->sDrive.sDrivePriority.bRampIncrement;
				else
					pPlanElement->sDrive.sDrivePriority.bRuntime_Priority = (instinctID)-1; // 0xff or 0xffff

				if (pPlanElement->sDrive.sReleaser.bRuntime_Released && pPlanElement->sDrive.sDrivePriority.bUrgencyMultiplier)
				{
					unsigned long lPriority = (unsigned long)pPlanElement->sDrive.sDrivePriority.bRuntime_Priority *
						(unsigned long)pPlanElement->sDrive.sDrivePriority.bUrgencyMultiplier;
					lPriority = lPriority / 32;
					if ((instinctID)-1 - pPlanElement->sDrive.sDrivePriority.bRuntime_Priority > lPriority)
						pPlanElement->sDrive.sDrivePriority.bRuntime_Priority += lPriority;
					else
						pPlanElement->sDrive.sDrivePriority.bRuntime_Priority = (instinctID)-1; // 0xff or 0xffff
				}
			}
		}
		pPlanElement = (PlanElement *)((unsigned char *)pPlanElement + nSize);
	}

	return INSTINCT_SUCCESS;
}

// read the sense via the Senses callback - primarily for testing
int Planner::readSense(const senseID nSense)
{
	return _pSenses->readSense(nSense);
}

// execute an action via the Actions callback - primarily for testing
unsigned char Planner::executeAction(const actionID nAction, const int nActionValue, const unsigned char bCheckForComplete)
{
	return _pActions->executeAction(nAction, nActionValue, bCheckForComplete);
}


/*	High level overview for Execution of a plan cycle. At each level the functionality involves searching for the correct child element to execute
	until we reach the lowest level (Action)

	High level pseudo code at overall plan level:
	1. If RAMP Model Processing enabled - update all plan priorities first
	2. Find Drive with highest priority that is available to be checked according to its frequency.
	3. Check it is still released, if not find next highest priority and repeat
	4. Note that multiple drives may have same priority. In this case the first one we come across is tested first
	5. Once we have a released Drive, process it by executing its child element. This can be an Action, an Action pattern, or a Competence
	6. Update the runtime cCounters

	High level pseudo code at Competence level:
	1. If CurrentElementID is set, then jump to that element and process it, else
	2. Find child element (a Competence Element) with highest priority and check if it is released, if not find next highest priority and repeat
	3. Note that multiple CE's may have same priority. In this case we must complete each one at that priority before moving on, provided each
	one's Releaser allows that. If we have set the UseORWithinCEGroup however, then we only need to successfully execute one Competence Element
	within a priority group. Therefore within one plan cycle we must keep trying untested releasers within the group until one succeeds.
	One of the real benefits of UseORWithinCEGroup is that the planner will test many releasers within a single plan cycle,
	so we can use this to build a case statement by using multiple CE's that reference the same Sense with different releaser parameters.
	4. Once we have a released CompetenceElement, process it
	5. If the CE fails or errors, then the Competence fails or errors and set CurrentElementID to zero. If UseORWithinCEGroup, then see if there
	are any untested releasers within the same priority group, as there may be more to try before we can fail. If one is found then set CurrentElementID
	for the next plan cycle and return In Progress. If the CE returns In Progress, then so does the Competence. If the CE returns SUCCESS then set the
	CurrentElementID to the next CE.
	6. Update the Runtime counters

	High level pseudo code at CompetenceElement level:
	1. Find the child element, and switch on its type (ActionPattern, Competence, Action)
	3. Update the Runtime counters

	High level pseudo code at ActionPattern level:
	1. If CurrentElementID is set, then jump to that element and process it, else
	2. Find child element (an ActionPatternElement) with lowest order that has not been completed and execute it
	3. Note that multiple APE's may have same order. In this case we must complete each one at that priority before moving on, with undefined order.
	4. If the APE fails or errors, then the AP fails or errors and set CurrentElementID to zero.
	If the APE returns In Progress, then so does the AP. If the APE returns Success then set the CurrentElementID to the next APE if one is found and
	return In Progress. If there are no more APE's then we are done, so the AP returns Success.
	5. Update the Runtime counters

	High level pseudo code at ActionPatternElement level:
	1. Find the child element, and switch on its type (ActionPattern, Competence, Action)
	2. Update the Runtime counters

	High level pseudo code at Action level:
	1. Execute the action implementation, passing the ActionValue parameter that is stored within the Action. This allows one action implementation to be
	invoked wby many actions, passing different parameters e.g. the implementation is Turn, and the ActionValue defines how many degrees to turn.
	2. If the action implementation returns In Progress, then set the RuntimeCheckComplete flag. This is used by the action implementation to recognise
	when an initial call is being made, as oposed to a continuation of an existing action request. If the underlying action implementation
	returns anything other than In Progress, clear the RuntimeCheckComplete flag.

*/

// Find Drive with highest priority. Check it is still released, if not find next highest priority and repeat
// this is the entry point for a plan cycle
unsigned char Planner::runPlan(void)
{
	PlanElement * pDriveNode;
	PlanElement * pDrive;
	instinctID nPriority;
	instinctID nDriveCount;
	int nSize;

	pDriveNode = _pPlan[INSTINCT_DRIVE];
	nDriveCount = _nNodeCount[INSTINCT_DRIVE];

	if (!pDriveNode || !nDriveCount)
		return false;

	nSize = sizeFromNodeType(INSTINCT_DRIVE);

	// first run over all the drives and clear the flag used to mark drives as tested in this cycle
	for (instinctID i = 0; i < nDriveCount; i++)
	{
		pDriveNode->sDrive.sDrivePriority.bRuntime_Checked = false;
		pDriveNode = (PlanElement *)((unsigned char *)pDriveNode + nSize);
	}


	// now keep running over drives, looking for the highest priority one that can be executed
	// note that multiple drives may have same priority. In this case the first one we come across is tested first
	do {
		pDriveNode = _pPlan[INSTINCT_DRIVE];
		pDrive = 0;
		nPriority = 0;

		for (instinctID i = 0; i < nDriveCount; i++)
		{
			if (!pDriveNode->sDrive.sDrivePriority.bRuntime_Checked)
			{
				// found an untested drive, so read and store it and its priority
				if (pDriveNode->sDrive.sDrivePriority.bRuntime_Priority > nPriority)
				{
					nPriority = pDriveNode->sDrive.sDrivePriority.bRuntime_Priority;
					pDrive = pDriveNode;
				}
			}

			pDriveNode = (PlanElement *)((unsigned char *)pDriveNode + nSize);
		}

		if (pDrive) // valid Drive node found
		{
			// we have found the highest priority Drive, so check if it can be released
			if (checkDriveFrequency(&pDrive->sDrive) &&
				(checkReleaser(pDrive, &pDrive->sDrive.sReleaser, &pDrive->sDrive) == INSTINCT_SUCCESS))
			{
				// if we can run this Drive, then other currently running drives become suspended
				// so record that fact in the drives
				pDriveNode = _pPlan[INSTINCT_DRIVE];
				for (instinctID i = 0; i < nDriveCount; i++)
				{
					if ((pDriveNode != pDrive) && (pDriveNode->sDrive.bRuntime_Status == INSTINCT_STATUS_RUNNING))
						pDriveNode->sDrive.bRuntime_Status = INSTINCT_STATUS_INTERRUPTED;
					pDriveNode = (PlanElement *)((unsigned char *)pDriveNode + nSize);
				}

				unsigned char bRtn = executeDrive(pDrive); // execute the Drive
				if (INSTINCT_RTN(bRtn) == INSTINCT_IN_PROGRESS)
					pDrive->sDrive.bRuntime_Status = INSTINCT_STATUS_RUNNING;
				else
				{
					pDrive->sDrive.bRuntime_Status = INSTINCT_STATUS_NOTRUNNING;
					// if the Drive is not running then its releaser must be assumed not to be released
					pDrive->sDrive.sReleaser.bRuntime_Released = false;
					// Reset the Runtime_Priority when the Drive completes if Ramping is enabled
					if (pDrive->sDrive.sDrivePriority.uiRampInterval && (INSTINCT_RTN(bRtn) == INSTINCT_SUCCESS))
					{
						pDrive->sDrive.sDrivePriority.bRuntime_Priority = pDrive->sDrive.sDrivePriority.bPriority;
					}
				}

				return bRtn;
			}
			else
			{
				// this drive is no longer released and so is not running
				pDrive->sDrive.bRuntime_Status = INSTINCT_STATUS_NOTRUNNING;
				pDrive->sDrive.sReleaser.bRuntime_Released = false;
				// mark this node as tested and move on
				pDrive->sDrive.sDrivePriority.bRuntime_Checked = true;
			}
		}
	} while (pDrive);

	return false;
}

// Execute a specific drive. A drive contains a single child element that may be an Action, ActionPattern or a Competence.
unsigned char Planner::executeDrive(PlanElement * pDrive)
{
	PlanElement *pElement;
	unsigned char bRtn;
	unsigned char bNodeType;

	// update the runtime counter for the drive
	countExecution(pDrive, INSTINCT_DRIVE);

	// get a pointer to the child element, and fill nNodeType with its type
	pElement = findChildAorAPorC(pDrive->sDrive.bRuntime_ChildID, &bNodeType);

	if (!pElement)
	{
		countError(pDrive, INSTINCT_DRIVE);
		return INSTINCT_ERROR; // only happens if plan structure is malformed
	}

	switch (bNodeType)
	{
	case INSTINCT_ACTION:
		bRtn = executeAction(pElement, pDrive);
		break;
	case INSTINCT_ACTIONPATTERN:
		bRtn = executeActionPattern(pElement, pDrive);
		break;
	case INSTINCT_COMPETENCE:
		bRtn = executeCompetence(pElement, pDrive);
		break;
	default:
		bRtn = INSTINCT_FAIL;
		break;
	}

	// check outcome of this cycle
	switch (INSTINCT_RTN(bRtn))
	{
	case INSTINCT_SUCCESS:
		countSuccess(pDrive, INSTINCT_DRIVE);
		break;

	case INSTINCT_IN_PROGRESS:
		countInProgress(pDrive, INSTINCT_DRIVE);
		break;

	case INSTINCT_FAIL:
		countFail(pDrive, INSTINCT_DRIVE);
		break;

	case INSTINCT_ERROR:
		countError(pDrive, INSTINCT_DRIVE);
		break;
	}

	return bRtn;
}

// Execute a specific Competence Element. May be from a Competence or a Drive
// A Competence Element (CE) may contain an Action (A), an Action Pattern (AP), or a Competence (C)
unsigned char Planner::executeCE(PlanElement *pCE, PlanElement *pDrive)
{
	unsigned char bRtn;
	PlanElement *pElement;
	unsigned char bNodeType;

	// update the runtime counter for this CE
	countExecution(pCE, INSTINCT_COMPETENCEELEMENT);

	// get a pointer to the child element, and fill nNodeType with its type
	pElement = findChildAorAPorC(pCE->sCompetenceElement.sParentChild.bRuntime_ChildID, &bNodeType);

	if (!pElement)
	{
		countError(pCE, INSTINCT_COMPETENCEELEMENT);
		return INSTINCT_ERROR; // only happens if plan structure is malformed
	}

	switch (bNodeType)
	{
	case INSTINCT_ACTION:
		bRtn = executeAction(pElement, pDrive);
		break;
	case INSTINCT_ACTIONPATTERN:
		bRtn = executeActionPattern(pElement, pDrive);
		break;
	case INSTINCT_COMPETENCE:
		bRtn = executeCompetence(pElement, pDrive);
		break;
	default:
		bRtn = INSTINCT_FAIL;
		break;
	}

	// update the runtime success counter
	switch (INSTINCT_RTN(bRtn))
	{
	case INSTINCT_SUCCESS:
		countSuccess(pCE, INSTINCT_COMPETENCEELEMENT);
		// reset the retry count
		pCE->sCompetenceElement.sRetry.bRuntime_RetryCount = 0;
		break;

	case INSTINCT_IN_PROGRESS:
		countInProgress(pCE, INSTINCT_COMPETENCEELEMENT);
		break;

	case INSTINCT_FAIL:
		// deal with CE retries by turning failures into INSTINCT_IN_PROGRESS
		if ((pCE->sCompetenceElement.sRetry.bRetryLimit > 0) &&
				(pCE->sCompetenceElement.sRetry.bRuntime_RetryCount < pCE->sCompetenceElement.sRetry.bRetryLimit))
		{
			pCE->sCompetenceElement.sRetry.bRuntime_RetryCount++;
			countInProgress(pCE, INSTINCT_COMPETENCEELEMENT);
			bRtn = INSTINCT_RTN_COMBINE(INSTINCT_IN_PROGRESS, INSTINCT_RTN_DATA(bRtn));
		}
		else
		{
			// we have reached the limit, so reset the retry counter and fail
			countFail(pCE, INSTINCT_COMPETENCEELEMENT);
			pCE->sCompetenceElement.sRetry.bRuntime_RetryCount = 0;
		}
		break;
	case INSTINCT_ERROR:
		countError(pCE, INSTINCT_COMPETENCEELEMENT);
		break;
	}

	return bRtn;
}

// Execute a specific Action. May be from a Drive (D), Competence Element (CE) or an Action Pattern Element (APE)
unsigned char Planner::executeAction(PlanElement *pAction, PlanElement *pDrive)
{
	unsigned char bRtn = 0;

	// update the runtime execution counter for the Action
	countExecution(pAction, INSTINCT_ACTION);
	bRtn = _pActions->executeAction(pAction->sAction.bActionID, pAction->sAction.nActionValue, pAction->sAction.bRuntime_CheckForComplete);

	switch(INSTINCT_RTN(bRtn))
	{
	case INSTINCT_SUCCESS:
		countSuccess(pAction, INSTINCT_ACTION);
		pAction->sAction.bRuntime_CheckForComplete = false;
		break;
	case INSTINCT_IN_PROGRESS:
		countInProgress(pAction, INSTINCT_ACTION);
		pAction->sAction.bRuntime_CheckForComplete = true;
		break;
	case INSTINCT_FAIL:
		countFail(pAction, INSTINCT_ACTION);
		pAction->sAction.bRuntime_CheckForComplete = false;
		break;
	case INSTINCT_ERROR:
		countError(pAction, INSTINCT_ACTION);
		pAction->sAction.bRuntime_CheckForComplete = false;
		break;
	}

	return bRtn;
}

// Execute a specific Action Pattern. May be from a Drive (D), Competence Element (CE) or an Action Pattern Element (APE)
unsigned char Planner::executeActionPattern(PlanElement *pActionPattern, PlanElement *pDrive)
{
	PlanElement * pAPE;
	unsigned char bRtn;

	// update the runtime counter for the Action pattern
	countExecution(pActionPattern, INSTINCT_ACTIONPATTERN);

	if (pActionPattern->sActionPattern.bRuntime_CurrentElementID > 0)
	{
		// The Action Pattern has already determined an APE to execute in a previous cycle.
		// If it can't be executed then fail and set the bRuntime_CurrentElementID to zero.
		pAPE = findElement(pActionPattern->sActionPattern.bRuntime_CurrentElementID, INSTINCT_ACTIONPATTERNELEMENT);
	}
	else // find the LOWEST Order APE and execute it
	{
		// RHW 28-01-16 We are starting the AP from the start, so clear down the state in all APE's
		clearAPECompletedFlags(pActionPattern->sReferences.bRuntime_ElementID);
		pAPE = findNextAPE(pActionPattern->sReferences.bRuntime_ElementID, 0);
	}

	if (!pAPE) // this should never happen
	{
		countError(pActionPattern, INSTINCT_ACTIONPATTERN);
		pActionPattern->sActionPattern.bRuntime_CurrentElementID = 0;
		clearAPECompletedFlags(pActionPattern->sReferences.bRuntime_ElementID);
		return INSTINCT_ERROR;
	}

	// we have found the APE to execute, so execute it
	bRtn = executeAPE(pAPE, pDrive);

	switch (INSTINCT_RTN(bRtn))
	{
	case INSTINCT_SUCCESS:// if we were successful then move on to next
		// remember that we've completed this APE
		pAPE->sActionPatternElement.bRuntime_Status = INSTINCT_RUNTIME_SUCCESS;

		// find next APE in this AP to execute and store its ID
		pAPE = findNextAPE(pActionPattern->sReferences.bRuntime_ElementID, pAPE->sActionPatternElement.bOrder);
		if (!pAPE) // nothing left to do, so we are done!
		{
			// this AP has succeeded! nothing more to be done except clear the Runtime_CurrentElementID,
			// clear all the bRuntime_Status flags and update the success counter
			countSuccess(pActionPattern, INSTINCT_ACTIONPATTERN);
			pActionPattern->sActionPattern.bRuntime_CurrentElementID = 0;
			clearAPECompletedFlags(pActionPattern->sReferences.bRuntime_ElementID);
		}
		else
		{
			// store the CE node to execute on the next cycle
			countInProgress(pActionPattern, INSTINCT_ACTIONPATTERN);
			pActionPattern->sActionPattern.bRuntime_CurrentElementID = pAPE->sReferences.bRuntime_ElementID;
			bRtn = INSTINCT_RTN_COMBINE(INSTINCT_IN_PROGRESS, INSTINCT_RTN_DATA(bRtn));
		}
		break;

	case INSTINCT_IN_PROGRESS: // call the same competence step next time
		countInProgress(pActionPattern, INSTINCT_ACTIONPATTERN);
		pActionPattern->sActionPattern.bRuntime_CurrentElementID = pAPE->sReferences.bRuntime_ElementID;
		break;

	case INSTINCT_FAIL:
		// clear the Runtime_CurrentElementID and clear all the bRuntime_Status flags
		countFail(pActionPattern, INSTINCT_ACTIONPATTERN);
		pActionPattern->sActionPattern.bRuntime_CurrentElementID = 0;
		clearAPECompletedFlags(pActionPattern->sReferences.bRuntime_ElementID);
		break;

	case INSTINCT_ERROR:
		// clear the Runtime_CurrentElementID and clear all the bRuntime_Status flags
		countError(pActionPattern, INSTINCT_ACTIONPATTERN);
		pActionPattern->sActionPattern.bRuntime_CurrentElementID = 0;
		clearAPECompletedFlags(pActionPattern->sReferences.bRuntime_ElementID);
		break;
	}

	return bRtn;
}

// Execute a specific Competence. May be from a Drive (D), Competence Element (CE) or an Action Pattern Element (APE)
unsigned char Planner::executeCompetence(PlanElement *pCompetence, PlanElement *pDrive)
{
	PlanElement * pCE;
	instinctID nCECount;
	unsigned char bRtn;

	pCE = _pPlan[INSTINCT_COMPETENCEELEMENT];
	nCECount = _nNodeCount[INSTINCT_COMPETENCEELEMENT];

	if (!pCE || !nCECount) // should never happen
		return INSTINCT_ERROR;

	// update the runtime counter for the Competence
	countExecution(pCompetence, INSTINCT_COMPETENCE);

	if (pCompetence->sCompetence.bRuntime_CurrentElementID > 0)
		bRtn = executeCompetenceSubsequent(pCompetence, pDrive);
	else
	{
		// RHW 28-01-16 we are starting the Competence from the start, so clear the state in the CE's
		clearCECompletedFlags(pCompetence->sReferences.bRuntime_ElementID);
		bRtn = executeCompetenceInitial(pCompetence, pDrive);
	}
	// check outcome of this cycle
	switch (INSTINCT_RTN(bRtn))
	{
	case INSTINCT_SUCCESS:
		// call success before we clear down all the state in the C and CE's
		countSuccess(pCompetence, INSTINCT_COMPETENCE);
		pCompetence->sCompetence.bRuntime_CurrentElementID = 0;
		clearCECompletedFlags(pCompetence->sReferences.bRuntime_ElementID);
		break;

	case INSTINCT_IN_PROGRESS:
		countInProgress(pCompetence, INSTINCT_COMPETENCE);
		break;

	case INSTINCT_ERROR:
		countError(pCompetence, INSTINCT_COMPETENCE);
		pCompetence->sCompetence.bRuntime_CurrentElementID = 0;
		clearCECompletedFlags(pCompetence->sReferences.bRuntime_ElementID);
		break;

	case INSTINCT_FAIL:
		countFail(pCompetence, INSTINCT_COMPETENCE);
		pCompetence->sCompetence.bRuntime_CurrentElementID = 0;
		clearCECompletedFlags(pCompetence->sReferences.bRuntime_ElementID);
		break;
	}

	return bRtn;
}


// called by executeCompetence and handles the situation where the bRuntime_CurrentElementID is zero
// i.e. the first time through the Competence plan execution
unsigned char Planner::executeCompetenceInitial(PlanElement *pCompetence, PlanElement *pDrive)
{
	PlanElement *pCE;
	instinctID nLastCEPriority = 0;
	instinctID nCEPriority;
	unsigned char bRtn = INSTINCT_FAIL;

	// find the highest level CE that is releasable and execute it
	while (pCE = findCEForReleaserCheck(pCompetence->sReferences.bRuntime_ElementID, nLastCEPriority))
	{
		nCEPriority = pCE->sCompetenceElement.sPriority.bPriority;

		// we have found the highest priority CE, so check if it can be released
		if (checkReleaser(pCE, &pCE->sCompetenceElement.sReleaser, &pDrive->sDrive) == INSTINCT_SUCCESS)
		{
			// we can run this CE
			bRtn = executeCE(pCE, pDrive); // execute the CE
			bRtn = processExecutedCE(pCE, pCompetence, pDrive, bRtn);
			break;
		}
		else
		{
			// this CE is not released, cycle round for next highest
			pCE->sCompetenceElement.bRuntime_Status = INSTINCT_RUNTIME_NOT_RELEASED;
			nLastCEPriority = nCEPriority;
		}
	};

	return bRtn;
}

// called by executeCompetence and handles the situation where the bRuntime_CurrentElementID is set
// i.e. we have already started competence execution
unsigned char Planner::executeCompetenceSubsequent(PlanElement *pCompetence, PlanElement *pDrive)
{
	PlanElement *pCE;
	instinctID nCEPriority;
	unsigned char bRtn = INSTINCT_FAIL;

	// The Competence has already determined a CE to execute in a previous cycle.
	// we need to check the CE can still be released, then execute it. If it can't be executed then
	// fail and set the bRuntime_CurrentElementID to zero.
	pCE = findElement(pCompetence->sCompetence.bRuntime_CurrentElementID, INSTINCT_COMPETENCEELEMENT);

	if (!pCE) // this should never happen
	{
		return INSTINCT_ERROR;
	}
	nCEPriority = pCE->sCompetenceElement.sPriority.bPriority;

	// clear the status flags for all CE's with same priority, if they were previously not released
	// as we are going to check them all again now in this cycle
	clearCENotReleasedStatus(pCompetence->sReferences.bRuntime_ElementID, nCEPriority);

	while (pCE)
	{
		// we have found the CE to execute, so check if it can be released or if it contains a running AP
		if (testCEForRunningAP(pCE) ||
			(checkReleaser(pCE, &pCE->sCompetenceElement.sReleaser, &pDrive->sDrive) == INSTINCT_SUCCESS))
		{
			// we can run this CE
			bRtn = executeCE(pCE, pDrive); // execute the CE
			bRtn = processExecutedCE(pCE, pCompetence, pDrive, bRtn);
			break;
		}
		else // the Releaser check has failed
		{
			instinctID bNextElementID = 0;
			pCE->sCompetenceElement.bRuntime_Status = INSTINCT_RUNTIME_NOT_RELEASED;

			bRtn = INSTINCT_FAIL;
			pCE = 0; // we are done with this CE for now
			// Are there others at this level with OR that we might need to try?
			if (!pCompetence->sCompetence.bUseORWithinCEGroup)
			{
				break; // one failure means we fail the Competence and start again.
			}
			else
			{
				// bUseORWithinCEGroup so we need to consider elements in the same priority group,
				// we need to get one of them to be released this cycle or we fail the Competence.
				if (pCE = findNextCE(pCompetence->sReferences.bRuntime_ElementID, nCEPriority, false, false))
				{
					if (pCE->sCompetenceElement.sPriority.bPriority == nCEPriority) // must be of the same priority or we have failed
					{
						pCompetence->sCompetence.bRuntime_CurrentElementID = pCE->sReferences.bRuntime_ElementID;
					}
					else
					{
						pCE = 0; // there are no CE's left to test at this Priority level, so fail
					}
				}
			}

		}
	}
	return bRtn;
}

// once a CE has ben executed, this function processes its return value and takes the appropriate next action
unsigned char Planner::processExecutedCE(PlanElement *pCE, PlanElement *pCompetence, PlanElement *pDrive, const unsigned char bRetVal)
{
	unsigned char bRtn = bRetVal;
	instinctID nCEPriority = pCE->sCompetenceElement.sPriority.bPriority;

	switch (INSTINCT_RTN(bRetVal))
	{
	case INSTINCT_SUCCESS: // if we were successful then move on
		// remember that we've completed this CE
		pCE->sCompetenceElement.bRuntime_Status = INSTINCT_RUNTIME_SUCCESS;

		// find next CE in this Competence to execute and store its ID
		// if bUseORWithinCEGroup then we move up to the next Priority, otherwise we search for same Priority upwards
		// include CE's that have previously failed the releaser test
		pCE = findNextCE(pCompetence->sReferences.bRuntime_ElementID, pCE->sCompetenceElement.sPriority.bPriority,
			pCompetence->sCompetence.bUseORWithinCEGroup, true);

		if (pCE) // there is more to do
		{
			// next cycle we need to attempt to execute this CE
			pCompetence->sCompetence.bRuntime_CurrentElementID = pCE->sReferences.bRuntime_ElementID;
			bRtn = INSTINCT_RTN_COMBINE(INSTINCT_IN_PROGRESS, INSTINCT_RTN_DATA(bRtn));
		}
		break;

	case INSTINCT_IN_PROGRESS:
		pCE->sCompetenceElement.bRuntime_Status = INSTINCT_RUNTIME_IN_PROGRESS;

		pCompetence->sCompetence.bRuntime_CurrentElementID = pCE->sReferences.bRuntime_ElementID;
		break;

	case INSTINCT_FAIL:
	case INSTINCT_ERROR:
		pCE->sCompetenceElement.bRuntime_Status = (INSTINCT_RTN(bRtn) == INSTINCT_FAIL) ? INSTINCT_RUNTIME_FAILED : INSTINCT_RUNTIME_ERROR;

		// For the OR functionality within a CE group, we should try another element at the same Priority if one exists, else fail the Competence
		if (pCompetence->sCompetence.bUseORWithinCEGroup)
		{
			// although the CE has failed, there may be another one at this priority level that we need to try
			// include CE's that may previously have failed the releaser check, because they may pass now
			if (pCE = findNextCE(pCompetence->sReferences.bRuntime_ElementID, pCE->sCompetenceElement.sPriority.bPriority,
				false, true))
			{
				if (pCE->sCompetenceElement.sPriority.bPriority == nCEPriority)
				{
					// there are more options with this priority group, so try the next one
					pCompetence->sCompetence.bRuntime_CurrentElementID = pCE->sReferences.bRuntime_ElementID;
					bRtn = INSTINCT_IN_PROGRESS;
				}
			}
		}
	}
	return bRtn;
}

// clear the status of all CE's with bParentCompetenceID and nCEPriority level, where status is set to INSTINCT_RUNTIME_NOT_RELEASED
unsigned char Planner::clearCENotReleasedStatus(const instinctID bParentCompetenceID, const instinctID nCEPriority)
{
	PlanElement *pCENode;
	PlanElement *pCE = 0;
	instinctID nCECount;
	int nSize;

	pCENode = _pPlan[INSTINCT_COMPETENCEELEMENT];
	nCECount = _nNodeCount[INSTINCT_COMPETENCEELEMENT];

	if (!pCENode || !nCECount) // should never happen
		return INSTINCT_ERROR;

	nSize = sizeFromNodeType(INSTINCT_COMPETENCEELEMENT);
	for (instinctID i = 0; i < nCECount; i++)
	{
		// if CE has our Competence as its parent
		if (pCENode->sCompetenceElement.sParentChild.bRuntime_ParentID == bParentCompetenceID)
		{
			// found a candidate CE, Check status and Priority
			if ((pCENode->sCompetenceElement.bRuntime_Status == INSTINCT_RUNTIME_NOT_RELEASED) &&
				(pCENode->sCompetenceElement.sPriority.bPriority == nCEPriority) )
			{
				pCENode->sCompetenceElement.bRuntime_Status = INSTINCT_RUNTIME_NOT_TESTED;
			}
		}
		pCENode = (PlanElement *)((unsigned char *)pCENode + nSize);
	}
	return INSTINCT_SUCCESS;

}


// check if a specific releaser can be released.
// pDrive points to the [parent] Drive, to check if it was interrupted, to determine if Flexible Latching should be applied
unsigned char Planner::checkReleaser(PlanElement *pPlanElement, ReleaserType * pReleaser, DriveType *pDrive)
{
	int nSenseValue;
	int nTriggerValue;
	int nHysteresis;
	unsigned char bReleased = INSTINCT_FAIL;

	// for INSTINCT_COMPARATOR_TR we don't need to read the sense. Just return SUCCESS or FAIL
	if (pReleaser->bComparator == INSTINCT_COMPARATOR_TR)
	{
		pReleaser->bRuntime_Released = true;
		countSense(pPlanElement, pReleaser, 0);
		return INSTINCT_SUCCESS;
	}
	else if (pReleaser->bComparator == INSTINCT_COMPARATOR_FL)
	{
		pReleaser->bRuntime_Released = false;
		countSense(pPlanElement, pReleaser, 0);
		return INSTINCT_FAIL;
	}

	// if the Drive has not been running, then the Releaser must be assumed to be not released
	if (pDrive->bRuntime_Status == INSTINCT_STATUS_NOTRUNNING)
	{
		pReleaser->bRuntime_Released = false;
	}

	nSenseValue = _pSenses->readSense(pReleaser->bSenseID);
	nTriggerValue = pReleaser->nSenseValue;
	// determine the correct Hysteresis value to use, dependent on whether the Drive has been interrupted
	nHysteresis = (pDrive->bRuntime_Status == INSTINCT_STATUS_INTERRUPTED) ?
		pReleaser->nSenseFlexLatchHysteresis :
		pReleaser->nSenseHysteresis;

	switch (pReleaser->bComparator)
	{
	case INSTINCT_COMPARATOR_EQ:
		if (nSenseValue == nTriggerValue)
			bReleased = INSTINCT_SUCCESS;
		break;
	case INSTINCT_COMPARATOR_NE:
		if (nSenseValue != nTriggerValue)
			bReleased = INSTINCT_SUCCESS;
		break;
	case INSTINCT_COMPARATOR_GT:
		// if sense was triggered on last cycle then use hysteresis
		if (pReleaser->bRuntime_Released)
			nTriggerValue -= nHysteresis;
		if (nSenseValue > nTriggerValue)
			bReleased = INSTINCT_SUCCESS;
		break;
	case INSTINCT_COMPARATOR_LT:
		// if sense was triggered on last cycle then use hysteresis
		if (pReleaser->bRuntime_Released)
			nTriggerValue += nHysteresis;
		if (nSenseValue < nTriggerValue)
			bReleased = INSTINCT_SUCCESS;
		break;
	default: // some incorrect comparator value
		bReleased = INSTINCT_ERROR;
	}
	pReleaser->bRuntime_Released = (bReleased == INSTINCT_SUCCESS) ? true : false;

	countSense(pPlanElement, pReleaser, nSenseValue);
	return bReleased;
}

// Check if a specific Drive can be run again yet. If it's running then keep it running
// Returns true if the timer has timed out. If the timer has timed out then reset it for next time!
// The timer is decremented via the processTimers() call, so the actual timing
// depends on how frequently processTimers() is called - might be plan cycles or real time
unsigned char Planner::checkDriveFrequency(DriveType *pDrive)
{
	if (pDrive->bRuntime_Status == INSTINCT_STATUS_RUNNING)
	{
		// I am already running
		return true;
	}
	else if (!pDrive->sFrequency.uiRuntime_IntervalCounter)
	{
		// start running and reset my interval counter for next time
		pDrive->sFrequency.uiRuntime_IntervalCounter = pDrive->sFrequency.uiInterval;
		return true;
	}
	return false;
}

// Tests whether this CE contains an AP, and if so whether the AP is running.
// It is used to inhibit reading the Releaser for the CE to allow the AP to complete.
unsigned char Planner::testCEForRunningAP(PlanElement *pCE)
{
	PlanElement *pAP;

	// look for a pointer to an AP element with this CE as parent, may not be an AP
	pAP = findElement(pCE->sCompetenceElement.sParentChild.bRuntime_ChildID, INSTINCT_ACTIONPATTERN);
	if (pAP)
	{
		if (pAP->sActionPattern.bRuntime_CurrentElementID)
		{
			return true;
		}
	}

	return false;
}

// This is a helper function encapsulating some complex search logic. It is used by execututeDrive and executeCompetence and it correctly finds
// the next CE element to attempt, based on the ID of the last attempted element and two flags. The flags must be correctly set to get the right result.
// It searches over Competence Elements and returns the lowest ordered item for a particular Parent CompetenceID that has not been attempted yet,
// but with the same or higher priority than the last element than was completed.
// However, If bNextLevel is set, then we cannot find a CE at the same priority level as the last one attempted.
// If bIncludeNotReleased is set, then we can also return items that have previously failed the releaser check.
// if no match then a null pointer is returned.
PlanElement * Planner::findNextCE(const instinctID bParentCompetenceID, const instinctID bLastElementPriority,
		const unsigned char bNextLevel, const unsigned char bIncludeNotReleased)
{
	PlanElement *pCENode;
	PlanElement *pCE = 0;
	instinctID nCEPriority;
	instinctID nCECount;
	int nSize;

	pCENode = _pPlan[INSTINCT_COMPETENCEELEMENT];
	nCECount = _nNodeCount[INSTINCT_COMPETENCEELEMENT];

	if (!pCENode || !nCECount) // should never happen
		return 0;

	nSize = sizeFromNodeType(INSTINCT_COMPETENCEELEMENT);
	nCEPriority = (instinctID)-1; // highest unsigned value 0xff or 0xffff
	for (instinctID i = 0; i < nCECount; i++)
	{
		// if CE has our Competence as its parent
		if (pCENode->sCompetenceElement.sParentChild.bRuntime_ParentID == bParentCompetenceID)
		{
			// found a candidate CE, so read and store its Order
			if (((pCENode->sCompetenceElement.bRuntime_Status == INSTINCT_RUNTIME_NOT_TESTED) || // must be untested or previously unreleased
				 (bIncludeNotReleased && (pCENode->sCompetenceElement.bRuntime_Status == INSTINCT_RUNTIME_NOT_RELEASED))) &&
				(pCENode->sCompetenceElement.sPriority.bPriority < nCEPriority) && // return the first one we find matching the criteria
				((bNextLevel && (pCENode->sCompetenceElement.sPriority.bPriority > bLastElementPriority)) || // must be higher priority if OR
				(!bNextLevel && (pCENode->sCompetenceElement.sPriority.bPriority >= bLastElementPriority)))) // must do all at same priority if AND
			{
				nCEPriority = pCENode->sCompetenceElement.sPriority.bPriority;
				pCE = pCENode;
			}
		}
		pCENode = (PlanElement *)((unsigned char *)pCENode + nSize);
	}
	return pCE;
}

// This is called only when first entering a Competence or Drive and determining which level to start execution.
// Start from bLastElementPriority and work down till we find an untested CE. If bLastElementPriority is 0 then start at the top.
PlanElement * Planner::findCEForReleaserCheck(const instinctID bParentCompetenceID, const instinctID bLastElementPriority)
{
	PlanElement *pCENode;
	PlanElement *pCE = 0;
	instinctID nCEPriority;
	instinctID nCECount;
	int nSize;

	pCENode = _pPlan[INSTINCT_COMPETENCEELEMENT];
	nCECount = _nNodeCount[INSTINCT_COMPETENCEELEMENT];

	if (!pCENode || !nCECount) // should never happen
		return 0;

	nSize = sizeFromNodeType(INSTINCT_COMPETENCEELEMENT);

	nCEPriority = 0;
	for (instinctID i = 0; i < nCECount; i++)
	{
		if (pCENode->sCompetenceElement.sParentChild.bRuntime_ParentID == bParentCompetenceID)
		{
			// we are only looking for NOT_TESTED nodes. If the Releaser check has failed then we will see NOTRELEASED on tested nodes
			// need to also consider untested nodes at same priority as the last one, as there may be more than one
			// at this stage we always find any node that is releasable within a group, and execute it.
			if ((pCENode->sCompetenceElement.bRuntime_Status == INSTINCT_RUNTIME_NOT_TESTED) &&
				(!bLastElementPriority || (pCENode->sCompetenceElement.sPriority.bPriority <= bLastElementPriority)))
			{
				// found an untested CE, so read and store its priority - use the first one we find at a given priority level
				if (pCENode->sCompetenceElement.sPriority.bPriority > nCEPriority)
				{
					nCEPriority = pCENode->sCompetenceElement.sPriority.bPriority;
					pCE = pCENode;
				}
			}
		}
		pCENode = (PlanElement *)((unsigned char *)pCENode + nSize);
	}
	return pCE;
}

// this is a helper function that will search over Action Pattern Elements and return the lowest ordered item for
// a particular Parent ActionPatternID that has not been completed in an order group and is greater than bLastElementOrder
PlanElement * Planner::findNextAPE(const instinctID bParentActionPatternID, const instinctID bLastElementOrder)
{
	PlanElement *pAPENode;
	PlanElement *pAPE = 0;
	instinctID nAPEOrder;
	instinctID nAPECount;
	int nSize;

	pAPENode = _pPlan[INSTINCT_ACTIONPATTERNELEMENT];
	nAPECount = _nNodeCount[INSTINCT_ACTIONPATTERNELEMENT];

	if (!pAPENode || !nAPECount) // should never happen
		return 0;

	nSize = sizeFromNodeType(INSTINCT_ACTIONPATTERNELEMENT);
	nAPEOrder = (instinctID)-1; // highest unsigned value 0xff or 0xffff
	for (instinctID i = 0; i < nAPECount; i++)
	{
		// if APE has our AP as its parent
		if (pAPENode->sActionPatternElement.sParentChild.bRuntime_ParentID == bParentActionPatternID)
		{
			// found a candidate APE, so read and store its Order
			if ((pAPENode->sActionPatternElement.bRuntime_Status == INSTINCT_RUNTIME_NOT_TESTED) &&
				(pAPENode->sActionPatternElement.bOrder < nAPEOrder) && // use the first one we find
				(pAPENode->sActionPatternElement.bOrder >= bLastElementOrder))
			{
				nAPEOrder = pAPENode->sActionPatternElement.bOrder;
				pAPE = pAPENode;
			}
		}
		pAPENode = (PlanElement *)((unsigned char *)pAPENode + nSize);
	}
	return pAPE;
}

// this is a helper function just to clear the bRuntime_Status flags
// of all CE's associated with a given ParentID
unsigned char Planner::clearCECompletedFlags(instinctID bParentID)
{
	PlanElement *pCENode;
	instinctID nCECount;
	int nSize;

	pCENode = _pPlan[INSTINCT_COMPETENCEELEMENT];
	nCECount = _nNodeCount[INSTINCT_COMPETENCEELEMENT];

	if (!pCENode || !nCECount) // should never happen
		return INSTINCT_ERROR;

	nSize = sizeFromNodeType(INSTINCT_COMPETENCEELEMENT);

	for (instinctID i = 0; i < nCECount; i++)
	{
		// find correct CE's based on ParentID
		if (pCENode->sCompetenceElement.sParentChild.bRuntime_ParentID == bParentID)
		{
			pCENode->sCompetenceElement.bRuntime_Status = INSTINCT_RUNTIME_NOT_TESTED;
		}
		pCENode = (PlanElement *)((unsigned char *)pCENode + nSize);
	}
	return INSTINCT_SUCCESS;
}

// this is a helper function just to clear the bRuntime_Status flags
// of all APE's associated with a given ParentID
unsigned char Planner::clearAPECompletedFlags(instinctID bParentID)
{
	PlanElement *pAPENode;
	instinctID nAPECount;
	int nSize;

	pAPENode = _pPlan[INSTINCT_ACTIONPATTERNELEMENT];
	nAPECount = _nNodeCount[INSTINCT_ACTIONPATTERNELEMENT];

	if (!pAPENode || !nAPECount) // should never happen
		return INSTINCT_ERROR;

	nSize = sizeFromNodeType(INSTINCT_ACTIONPATTERNELEMENT);

	for (instinctID i = 0; i < nAPECount; i++)
	{
		// find correct CE's based on ParentID
		if (pAPENode->sActionPatternElement.sParentChild.bRuntime_ParentID == bParentID)
		{
			pAPENode->sActionPatternElement.bRuntime_Status = INSTINCT_RUNTIME_NOT_TESTED;
		}
		pAPENode = (PlanElement *)((unsigned char *)pAPENode + nSize);
	}
	return INSTINCT_SUCCESS;
}

// Execute a specific Action Pattern Element. Must be from an Action Pattern (AP)
// An Action Pattern Element (APE) may contain an Action (A), an Action Pattern (AP), or a Competence (C)
unsigned char Planner::executeAPE(PlanElement *pAPE, PlanElement *pDrive)
{
	PlanElement *pElement;
	unsigned char bNodeType;
	unsigned char bRtn = 0;

	// update the runtime execution counter for the Action Pattern Element
	countExecution(pAPE, INSTINCT_ACTIONPATTERNELEMENT);

	// get a pointer to the child element, and fill nNodeType with its type
	pElement = findChildAorAPorC(pAPE->sActionPatternElement.sParentChild.bRuntime_ChildID, &bNodeType);

	if (!pElement)
	{
		countError(pAPE, INSTINCT_ACTIONPATTERNELEMENT);
		return INSTINCT_ERROR; // only happens if plan structure is malformed
	}

	switch (bNodeType)
	{
	case INSTINCT_ACTION:
		bRtn = executeAction(pElement, pDrive);
		break;
	case INSTINCT_ACTIONPATTERN:
		bRtn = executeActionPattern(pElement, pDrive);
		break;
	case INSTINCT_COMPETENCE:
		bRtn = executeCompetence(pElement, pDrive);
		break;
	default:
		bRtn = INSTINCT_FAIL;
		break;
	}

	// update the runtime success counter
	switch (INSTINCT_RTN(bRtn))
	{
	case INSTINCT_SUCCESS:
		countSuccess(pAPE, INSTINCT_ACTIONPATTERNELEMENT);
		break;
	case INSTINCT_IN_PROGRESS:
		countInProgress(pAPE, INSTINCT_ACTIONPATTERNELEMENT);
		break;
	case INSTINCT_FAIL:
		countFail(pAPE, INSTINCT_ACTIONPATTERNELEMENT);
		break;
	case INSTINCT_ERROR:
		countError(pAPE, INSTINCT_ACTIONPATTERNELEMENT);
		break;
	}

	return bRtn;
}

} // /namespace Instinct