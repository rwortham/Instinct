//  Instinct Element Name to instinctID mapping
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

Names::Names(const unsigned int uiBufferSize)
{
  pElementNameBuffer = NULL;

  pElementNameBuffer = (ElementNameBufferType *)malloc(uiBufferSize);

  if (pElementNameBuffer)
  {
    pElementNameBuffer->uiBuffLen = uiBufferSize;
    pElementNameBuffer->bEntryCount = 0;
  }
}

unsigned char Names::addElementName(const instinctID bRuntime_ElementID, char *pElementName)
{
  if (!pElementNameBuffer || !pElementName)
  return false;

  ElementNameEntryType *pEntry = pElementNameBuffer->sEntry;

  for (instinctID i=0; i < pElementNameBuffer->bEntryCount; i++)
  {
    // check if ElementID is already in the list, and return true
    if (pEntry->bRuntime_ElementID == bRuntime_ElementID)
      return true;
    pEntry = (ElementNameEntryType *)((char *)pEntry + sizeof(instinctID) + strlen(pEntry->szName)+1);
  }

  // check if there is room for another entry
  if ((void *)((char *)pEntry + sizeof(instinctID) + strlen(pElementName)) > (void *)((char *)pElementNameBuffer + pElementNameBuffer->uiBuffLen))
  {
    return false;
  }

  pEntry->bRuntime_ElementID = bRuntime_ElementID;
  strcpy(pEntry->szName, pElementName);
  pElementNameBuffer->bEntryCount++;

  return true;
}

char * Names::getElementName(const instinctID bRuntime_ElementID)
{
  if (!pElementNameBuffer)
  return NULL;

  ElementNameEntryType *pEntry = pElementNameBuffer->sEntry;

  for (instinctID i=0; i < pElementNameBuffer->bEntryCount; i++)
  {
    if (pEntry->bRuntime_ElementID == bRuntime_ElementID)
      return pEntry->szName;

    pEntry = (ElementNameEntryType *)((char *)pEntry + sizeof(instinctID) + strlen(pEntry->szName)+1);
  }

  return NULL;
}

instinctID Names::getElementID(const char *pName)
{
	if (!pElementNameBuffer || !pName)
		return NULL;

	ElementNameEntryType *pEntry = pElementNameBuffer->sEntry;

	for (instinctID i = 0; i < pElementNameBuffer->bEntryCount; i++)
	{
		if (!strcmp(pEntry->szName, pName))
			return pEntry->bRuntime_ElementID;

		pEntry = (ElementNameEntryType *)((char *)pEntry + sizeof(instinctID) + strlen(pEntry->szName) + 1);
	}

	return 0;
}

unsigned char Names::clearElementNames(void)
{
  if (!pElementNameBuffer)
    return false;
  pElementNameBuffer->bEntryCount = 0;
  return true;
}

instinctID Names::elementNameCount(void)
{
  return pElementNameBuffer ? pElementNameBuffer->bEntryCount : 0;
}

unsigned char * Names::elementBuffer(void)
{
  return (unsigned char *)pElementNameBuffer;
}

unsigned int Names::elementBufferSize(void)
{
  return pElementNameBuffer ? pElementNameBuffer->uiBuffLen : 0;
}

instinctID Names::maxElementNameID(void)
{
	instinctID bMaxID = 0;

	if (!pElementNameBuffer)
		return 0;

	ElementNameEntryType *pEntry = pElementNameBuffer->sEntry;

	for (instinctID i=0; i < pElementNameBuffer->bEntryCount; i++)
	{
		if (pEntry->bRuntime_ElementID > bMaxID)
			bMaxID = pEntry->bRuntime_ElementID;

		pEntry = (ElementNameEntryType *)((char *)pEntry + sizeof(instinctID) + strlen(pEntry->szName)+1);
	}

	return bMaxID;
}

} // /namespace Instinct