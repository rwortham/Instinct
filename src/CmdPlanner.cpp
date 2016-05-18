//  Instinct Reactive Planning Library. The CmdPlanner adds the ability
//  to interact with the Plan via a textual command line
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

CmdPlanner::CmdPlanner(instinctID *pPlanSize, Senses *pSenses, Actions *pActions, Monitor *pMonitor)
		: Planner(pPlanSize, pSenses, pActions, pMonitor)
{
}

// return multiple lines of help text explaining all commands. Note lines separated by "!"
// maximum line length is 80 chars
const char * CmdPlanner::help(void)
{
static const char PROGMEM szHelp[] = { "PLAN Commands:!"
"A{Add plan element}|U{Update plan element}|M{Config Monitor}|R{Reset plan}]!"
"                                                         [parameter values]!"
"A - add elements to the existing plan!"
"  A [D{Drive}|C{Competence}|A{Action}|P{Action Pattern}|E{Competence Element}|!"
"     L{ActionPattern Element}] [parameters]!"
"    The A D command has 12 parameters as below:!"
"        A D Runtime_ElementID Runtime_ChildID Priority uiInterval SenseID!"
"            Comparator SenseValue SenseHysteresis SenseFlexLatchHysteresis!"
"            RampIncrement UrgencyMultiplier RampInterval!"
"    The A C command has 2 parameters:!"
"        A C Runtime_ElementID UseORWithinCEGroup!"
"    The A A command has 3 parameters:!"
"        A A Runtime_ElementID ActionID ActionValue!"
"    The A P command has just one parameter:!"
"        A P Runtime_ElementID!"
"    The A E command has 10 parameters:!"
"        A E Runtime_ElementID Runtime_ParentID Runtime_ChildID Priority!"
"            RetryLimit SenseID Comparator SenseValue SenseHysteresis!"
"            SenseFlexLatchHysteresis!"
"      The A L command has 4 parameters:!"
"        A L Runtime_ElementID Runtime_ParentID Runtime_ChildID Order!"
"D - display a given node, or the highest element ID!"
"  D [N{display plan settings for a node}|C{display counters for a node}|!"
"     H{Highest node ID}]!"
"      The D N and D C commands have 1 parameter as below:!"
"          D {N|C} Runtime_ElementID!"
"      The D H command takes no parameters.!"
"U - command not yet supported. Will allow update of individual nodes!"
"M - Update the monitor flags for a specific node, or the global flags!"
"  M [N{Node ID}|G{Global flags}]!"
"      The M N command has 7 parameters!"
"          M N Runtime_ElementID MonitorExecuted MonitorSuccess MonitorPending!"
"              MonitorFail MonitorError MonitorSense e.g. M N 27 1 1 0 1 1 1!"
"      The M G command has 6 parameters!"
"          M G MonitorExecuted MonitorSuccess MonitorPending MonitorFail!"
"              MonitorError MonitorSense e.g. M N  0 1 0 0 0 1!"
"R - Clear the plan and initialise a new one!"
"  R [C{clear plan}|I{clear plan and initialise new one}]!"
"      The R C command takes no parameters!"
"      The R I command takes 6 parameters!"
"          R I COUNT_ACTIONPATTERN COUNT_ACTIONPATTERNELEMENT COUNT_COMPETENCE!"
"              COUNT_COMPETENCEELEMENT COUNT_DRIVE COUNT_ACTION!"
"              e.g. R I 0 0 1 10 2 20!"
"S - return the size of the plan into a string buffer!"
"  S [C{return node counts}|S{return total plan size}]!"
"      The S C and S S commands take no parameters!"
"I - Set/return the ID of the plan!"
"  I [S{set the plan ID}|R{return the plan ID}]!"
"      The I S command takes 1 parameter!"
"          I S [PlanID]!"
"      The I R command takes no parameters!"
	};

	return szHelp;
}

unsigned char CmdPlanner::executeCommand(const char * pCmd, char *pRtnBuff, const int nRtnBuffLen)
{
	unsigned char bSuccess = false;
	char cCmd[2];
	int nIntArray[12];
	static const char PROGMEM szFmt[] = { "%c %c %i %i %i %i %i %i %i %i %i %i %i %i" };

	memset(nIntArray, 0, sizeof(nIntArray)); // set all the parameters to zero before reading
	int nRtn = sscanf_P(pCmd, szFmt, &cCmd[0], &cCmd[1], &nIntArray[0], &nIntArray[1], &nIntArray[2], &nIntArray[3],
		&nIntArray[4], &nIntArray[5], &nIntArray[6], &nIntArray[7], &nIntArray[8], &nIntArray[9], &nIntArray[10], &nIntArray[11]);

	if (pRtnBuff) // clear the return message buffer
		*pRtnBuff = 0;

	// we did not read the first two command params, so just force the FAIL condition
	if (nRtn < 2)
	{
		cCmd[0] = 0;
	}

	switch (cCmd[0])
	{
	case 'A': // we are adding a node
		switch (cCmd[1])
		{
		case 'D': // for a Drive we need 12 parameters
				// Runtime_ElementID Runtime_ChildID Priority uiInterval SenseID Comparator
				// SenseValue SenseHysteresis SenseFlexLatchHysteresis RampIncrement UrgencyMultiplier RampInterval
			if (nRtn == 14)
				bSuccess = addDrive((instinctID)nIntArray[0], (instinctID)nIntArray[1],(instinctID)nIntArray[2], nIntArray[3], (instinctID)nIntArray[4], (unsigned char)nIntArray[5],
				nIntArray[6], nIntArray[7], nIntArray[8], (instinctID)nIntArray[9], (instinctID)nIntArray[10], (instinctID)nIntArray[11]);
			break;
		case 'C': // for a Competence we need 2 parameters
			if (nRtn == 4)
				bSuccess = addCompetence((instinctID)nIntArray[0], (unsigned char)nIntArray[1]);
			break;
		case 'A': // for an action we need 3 parameters
			if (nRtn == 5)
				bSuccess = addAction((instinctID)nIntArray[0], (instinctID)nIntArray[1], nIntArray[2]);
			break;
		case 'P':
			if (nRtn == 3) // for an ActionPattern we need 1 parameter
				bSuccess = addActionPattern((instinctID)nIntArray[0]);
			break;
		case 'E':
			if (nRtn == 12)// for a CompetenceElement we need 10 parameters
				bSuccess = addCompetenceElement((instinctID)nIntArray[0], (instinctID)nIntArray[1], nIntArray[2], (instinctID)nIntArray[3], (unsigned char)nIntArray[4],
				(instinctID)nIntArray[5], (unsigned char)nIntArray[6], nIntArray[7], (instinctID)nIntArray[8], (instinctID)nIntArray[9]);
			break;
		case 'L':
			if (nRtn == 6)// for an ActionPatternElement we need 4 parameters
				bSuccess = addActionPatternElement((instinctID)nIntArray[0], (instinctID)nIntArray[1], (instinctID)nIntArray[2], (instinctID)nIntArray[3]);
			break;
		}
		break;
	case 'U':
		// TODO
		break;
	case 'M': // configure the node monitoring
		switch (cCmd[1])
		{
		case 'N':
			if (nRtn == 9) // to config monitoring for a single node we need 7 parameters
				bSuccess = monitorNode((instinctID)nIntArray[0], (unsigned char)nIntArray[1], (unsigned char)nIntArray[2], (unsigned char)nIntArray[3],
							(unsigned char)nIntArray[4], (unsigned char)nIntArray[5], (unsigned char)nIntArray[6]);
			break;
		case 'G':
			if (nRtn == 8) // to config monitoring globally we need 6 parameters
			{
				setGlobalMonitorFlags((unsigned char)nIntArray[0], (unsigned char)nIntArray[1], (unsigned char)nIntArray[2], (unsigned char)nIntArray[3],
					(unsigned char)nIntArray[4], (unsigned char)nIntArray[5]);
				bSuccess = true;
			}
			break;
		}
		break;
	case 'R': // we are resetting the whole plan
		instinctID nPlanSize[INSTINCT_NODE_TYPES];
		memset(nPlanSize, 0, sizeof(nPlanSize));
		switch (cCmd[1])
		{
		case 'C':
			if (nRtn == 2) // just clear the plan
				bSuccess = initialisePlan(nPlanSize);
			break;
		case 'I':
			if (nRtn == 8) // to config monitoring globally we need 6 parameters
			{
				for (int i = 0; i < INSTINCT_NODE_TYPES; i++)
					nPlanSize[i] = (instinctID)nIntArray[i];
				bSuccess = initialisePlan(nPlanSize);
			}
			break;
		}
		break;
	case 'S': // return size of plan
		if (pRtnBuff && (nRtnBuffLen >(11 * 4)))
		{
			switch (cCmd[1])
			{
			case 'C': // return counts
				for (int i = 0; i < INSTINCT_NODE_TYPES; i++)
				{
					int nStrLen = strlen(pRtnBuff);
					static const char PROGMEM szFmt[] = {"%u "};
					snprintf_P(pRtnBuff + nStrLen, nRtnBuffLen - nStrLen, szFmt, (unsigned int)planSize(i));
				}
				bSuccess = true;
				break;
			case 'S': // return total size
				unsigned int uiSize = planUsage(0);
				static const char PROGMEM szFmt[] = {"%u"};
				snprintf_P(pRtnBuff, nRtnBuffLen, szFmt, uiSize);
				bSuccess = true;
				break;
			}
		}
		break;
	case 'D': // display node info
		if (pRtnBuff && (nRtnBuffLen > (11 * 4)))
		{
			switch (cCmd[1])
			{
			case 'N': // display the given node ID
				if (nRtn == 3) // we need the node ID
				{
					bSuccess = displayNode(pRtnBuff, nRtnBuffLen, (instinctID)nIntArray[0]);
				}
				break;
			case 'C': // display the counters for the given node ID
				if (nRtn == 3) // we need the node ID
				{
					bSuccess = displayNodeCounters(pRtnBuff, nRtnBuffLen, (instinctID)nIntArray[0]);
				}
				break;
			case 'H': // return highest node count
				static const char PROGMEM szFmt[] = {"%u"};
				snprintf_P(pRtnBuff, nRtnBuffLen, szFmt, (unsigned int)maxElementID());
				bSuccess = true;
				break;
			}
		}
		break;
	case 'I': // set or return the plan ID
		if (pRtnBuff && (nRtnBuffLen > 6))
		{
			switch (cCmd[1])
			{
			case 'S': // set the PlanID
				_nPlanID = nIntArray[0];
				bSuccess = true;
				break;
			case 'R': // read the PlanID
				static const char PROGMEM szFmt[] = {"%i"};
				snprintf_P(pRtnBuff, nRtnBuffLen, szFmt, _nPlanID);
				bSuccess = true;
				break;
			}
		}
		break;
	}

	if (pRtnBuff && (nRtnBuffLen >= 5))
	{
		if (!strlen(pRtnBuff)) // if command has not used buffer then return OK or FAIL
			strncat(pRtnBuff, bSuccess ? "OK" : "FAIL", nRtnBuffLen);
	}

	return bSuccess;
}

// Display the node as a single line of text suitable for reading back into the plan via executeCommand()
unsigned char CmdPlanner::displayNode(char *pStrBuff, const int nBuffLen, const instinctID nElementID)
{
	PlanNode sPlanNode;

	// make sure there is a buffer and its big enough
	if (!pStrBuff || (nBuffLen < 80) || !nElementID || !getNode(&sPlanNode, nElementID))
		return false;
	return displayNode(pStrBuff, nBuffLen, &sPlanNode);
}

// Display the given node as a single line of text
unsigned char CmdPlanner::displayNode(char *pStrBuff, const int nBuffLen, const PlanNode *pPlanNode)
{
	static const char PROGMEM szFmt1[] = { "A D %u %u %u %u %u %u %i %i %i %u %u %u" };
	static const char PROGMEM szFmt2[] = { "A E %u %u %u %u %u %u %u %i %i %i" };
	static const char PROGMEM szFmt3[] = { "A P %u " };
	static const char PROGMEM szFmt4[] = { "A L %u %u %u %u" };
	static const char PROGMEM szFmt5[] = { "A A %u %u %i" };
	static const char PROGMEM szFmt6[] = { "A C %u %u" };

	switch (pPlanNode->bNodeType)
	{
	case INSTINCT_DRIVE:
		/*			unsigned char addDrive(const instinctID bRuntime_ElementID, const instinctID bPriority, const unsigned int uiInterval,
		const senseID bSenseID, const unsigned char bComparator, const int nSenseValue,
		const int nSenseHysteresis, const int nSenseFlexLatchHysteresis,
		const instinctID bRampIncrement, const instinctID bUrgencyMultiplier, const instinctID uiRampInterval);
		*/
		snprintf_P(pStrBuff, nBuffLen, szFmt1, (unsigned int)pPlanNode->sElement.sReferences.bRuntime_ElementID,
			(unsigned int)pPlanNode->sElement.sDrive.bRuntime_ChildID,
			(unsigned int)pPlanNode->sElement.sDrive.sDrivePriority.bPriority,
			(unsigned int)pPlanNode->sElement.sDrive.sFrequency.uiInterval,
			(unsigned int)pPlanNode->sElement.sDrive.sReleaser.bSenseID,
			(unsigned int)pPlanNode->sElement.sDrive.sReleaser.bComparator,
			(int)pPlanNode->sElement.sDrive.sReleaser.nSenseValue,
			(int)pPlanNode->sElement.sDrive.sReleaser.nSenseHysteresis,
			(int)pPlanNode->sElement.sDrive.sReleaser.nSenseFlexLatchHysteresis,
			(unsigned int)pPlanNode->sElement.sDrive.sDrivePriority.bRampIncrement,
			(unsigned int)pPlanNode->sElement.sDrive.sDrivePriority.bUrgencyMultiplier,
			(unsigned int)pPlanNode->sElement.sDrive.sDrivePriority.uiRampInterval);
		break;
	case INSTINCT_COMPETENCEELEMENT:
		/*			unsigned char addCompetenceElement(const instinctID bRuntime_ElementID, const instinctID bRuntime_ParentID,
		const instinctID bRuntime_ChildID, const instinctID bPriority, unsigned char bRetryLimit,
		const senseID bSenseID, const unsigned char bComparator, const int nSenseValue,
		const int nSenseHysteresis, const int nSenseFlexLatchHysteresis);
		*/
		snprintf_P(pStrBuff, nBuffLen, szFmt2,
			(unsigned int)pPlanNode->sElement.sReferences.bRuntime_ElementID,
			(unsigned int)pPlanNode->sElement.sCompetenceElement.sParentChild.bRuntime_ParentID,
			(unsigned int)pPlanNode->sElement.sCompetenceElement.sParentChild.bRuntime_ChildID,
			(unsigned int)pPlanNode->sElement.sCompetenceElement.sPriority.bPriority,
			(unsigned int)pPlanNode->sElement.sCompetenceElement.sRetry.bRetryLimit,
			(unsigned int)pPlanNode->sElement.sCompetenceElement.sReleaser.bSenseID,
			(unsigned int)pPlanNode->sElement.sCompetenceElement.sReleaser.bComparator,
			(int)pPlanNode->sElement.sCompetenceElement.sReleaser.nSenseValue,
			(int)pPlanNode->sElement.sCompetenceElement.sReleaser.nSenseHysteresis,
			(int)pPlanNode->sElement.sCompetenceElement.sReleaser.nSenseFlexLatchHysteresis);
		break;
	case INSTINCT_ACTIONPATTERN:
		/*			unsigned char addActionPattern(const instinctID bRuntime_ElementID);
		*/
		snprintf_P(pStrBuff, nBuffLen, szFmt3, (unsigned int)pPlanNode->sElement.sReferences.bRuntime_ElementID);
		break;
	case INSTINCT_ACTIONPATTERNELEMENT:
		/*		unsigned char addActionPatternElement(const instinctID bRuntime_ElementID, const instinctID bRuntime_ParentID,
		const instinctID bRuntime_ChildID,const instinctID bOrder);
		*/
		snprintf_P(pStrBuff, nBuffLen, szFmt4, (unsigned int)pPlanNode->sElement.sReferences.bRuntime_ElementID,
			(unsigned int)pPlanNode->sElement.sActionPatternElement.sParentChild.bRuntime_ParentID,
			(unsigned int)pPlanNode->sElement.sActionPatternElement.sParentChild.bRuntime_ChildID,
			(unsigned int)pPlanNode->sElement.sActionPatternElement.bOrder);
		break;
	case INSTINCT_ACTION:
		/*		unsigned char addAction(const instinctID bRuntime_ElementID, const actionID bActionID, const int nActionValue);
		*/
		snprintf_P(pStrBuff, nBuffLen, szFmt5, (unsigned int)pPlanNode->sElement.sReferences.bRuntime_ElementID,
			(unsigned int)pPlanNode->sElement.sAction.bActionID,
			(int)pPlanNode->sElement.sAction.nActionValue);
		break;
	case INSTINCT_COMPETENCE:
		/*			unsigned char addCompetence(const instinctID bRuntime_ElementID);
		*/
		snprintf_P(pStrBuff, nBuffLen, szFmt6, (unsigned int)pPlanNode->sElement.sReferences.bRuntime_ElementID,
			(unsigned int)pPlanNode->sElement.sCompetence.bUseORWithinCEGroup);
		break;
	}

	return true;
}

// Display the node counters as a single line of text suitable for reading back into the plan via executeCommand()
unsigned char CmdPlanner::displayNodeCounters(char *pStrBuff, const int nBuffLen, const instinctID nElementID)
{
	PlanNode sPlanNode;

	// make sure there is a buffer and its big enough
	if (!pStrBuff || (nBuffLen < 80) || !nElementID || !getNode(&sPlanNode, nElementID))
		return false;

	return displayNodeCounters(pStrBuff, nBuffLen, &sPlanNode);
}

// Display the given node counters as a single line of text
unsigned char CmdPlanner::displayNodeCounters(char *pStrBuff, const int nBuffLen, const PlanNode *pPlanNode)
{
	// for all node types, display the ID, ExecutionCount and SuccessCount
	static const char PROGMEM szFmt[] = {"%u %u %u "};
	snprintf_P(pStrBuff, nBuffLen, szFmt, (unsigned int)pPlanNode->sElement.sReferences.bRuntime_ElementID,
		(unsigned int)pPlanNode->sElement.sCounters.uiRuntime_ExecutionCount,
		(unsigned int)pPlanNode->sElement.sCounters.uiRuntime_SuccessCount);

	// add some extra runtime values to display, depending on the node type
	int nLen = strlen(pStrBuff);
	int nBuffLeft = nBuffLen - nLen;
	pStrBuff += nLen;

	static const char PROGMEM szFmt1[] = {"%u %u %u %u"};
	static const char PROGMEM szFmt2[] = {"%u"};

	switch (pPlanNode->bNodeType)
	{
	case INSTINCT_DRIVE:
		snprintf_P(pStrBuff, nBuffLeft, szFmt1,
			(unsigned int)pPlanNode->sElement.sDrive.sDrivePriority.uiRuntime_RampIntervalCounter,
			(unsigned int)pPlanNode->sElement.sDrive.sFrequency.uiRuntime_IntervalCounter,
			(unsigned int)pPlanNode->sElement.sDrive.sDrivePriority.bRuntime_Priority,
			(unsigned int)pPlanNode->sElement.sDrive.bRuntime_Status);
		break;
	case INSTINCT_COMPETENCEELEMENT:
		snprintf_P(pStrBuff, nBuffLeft, szFmt2, (unsigned int)pPlanNode->sElement.sCompetenceElement.bRuntime_Status);
		break;
	case INSTINCT_ACTIONPATTERN:
		snprintf_P(pStrBuff, nBuffLeft, szFmt2, (unsigned int)pPlanNode->sElement.sActionPattern.bRuntime_CurrentElementID);
		break;
	case INSTINCT_ACTIONPATTERNELEMENT:
		snprintf_P(pStrBuff, nBuffLeft, szFmt2, (unsigned int)pPlanNode->sElement.sActionPatternElement.bRuntime_Status);
		break;
	case INSTINCT_ACTION:
		snprintf_P(pStrBuff, nBuffLeft, szFmt2, (unsigned int)pPlanNode->sElement.sAction.bRuntime_CheckForComplete);
		break;
	case INSTINCT_COMPETENCE:
		snprintf_P(pStrBuff, nBuffLeft, szFmt2, (unsigned int)pPlanNode->sElement.sCompetence.bRuntime_CurrentElementID);
		break;
	}

	return true;
}

// Display the given releaser as a single line of text
unsigned char CmdPlanner::displayReleaser(char *pStrBuff, const int nBuffLen, const ReleaserType *pReleaser)
{
	static const char PROGMEM szFmt[] = { "%u %u %i %i %i %u" };
	snprintf_P(pStrBuff, nBuffLen, szFmt, (unsigned int)pReleaser->bSenseID,
		(unsigned int)pReleaser->bComparator,
		(int)pReleaser->nSenseValue,
		(int)pReleaser->nSenseHysteresis,
		(int)pReleaser->nSenseFlexLatchHysteresis,
		(unsigned int)pReleaser->bRuntime_Released);

	return true;
}

} // /namespace Instinct