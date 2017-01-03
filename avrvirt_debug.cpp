/* Copyright (c) 2017, Attila Laszlo Agas (http://www.hwelectrons.com)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the copyright holder nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <avrvirt.h>
#include <stdio.h>

#include "avrvirt_debug.h"

#ifdef AVRVIRT_DEBUG
namespace avrvirt
{
//-------------------------------------------------------------------------    
uint8_t getRrIndex(const avr_word_t opcode)
{
  uint8_t index = opcode.byte.lo & (uint8_t)0x0Fu;
  if (opcode.byte.hi & (uint8_t)0x02u)
  {
    index |= (uint8_t)0x10u;
  }
  return index;
}
//-------------------------------------------------------------------------    
uint8_t getRdIndex(const avr_word_t opcode)
{
  uint8_t index = shr4(opcode.byte.lo);
  if (opcode.byte.hi & (uint8_t)0x01u)
  {
    index |= (uint8_t)0x10u;
  }
  return index;
}
//-------------------------------------------------------------------------    
uint8_t getRdUpperIndex(const avr_word_t opcode)
{
  return shr4(opcode.byte.lo) + (uint8_t)16U;
}
//-------------------------------------------------------------------------    
uint8_t getRrUpperIndex(const avr_word_t opcode)
{
  return (opcode.byte.lo & (uint8_t)0x0Fu) + (uint8_t)16U;
}
//-------------------------------------------------------------------------
uint8_t getRdUpper4PairsIndex(const avr_word_t opcode)
{
  uint8_t iOffset = opcode.byte.lo;
  __asm__ __volatile__
  ("swap %0\n\t"
   "lsl %0\n\t"
   "andi %0,6\n\t" : "+d" (iOffset) :);
  return iOffset+24; 
}
//-------------------------------------------------------------------------
uint8_t getIndexRegIndex(const avr_word_t opcode)
{
  switch((opcode.byte.lo >> (uint8_t)2u) & (uint8_t)0b11u)
  {
    case 0b00u:
      return 30;
      break;
    case 0b10u:
      return 28;
      break;
    case 0b11u:
      return 26;
      break;
  }
}
//-------------------------------------------------------------------------
const char* niceSREG(uint8_t sreg)
{
  static char nice[11]; 
  static const char nice_pattern[] PROGMEM = "ITHSVNZC";
  char empty='-';
  nice[0]='[';
  nice[9]=']';
  nice[10]=0;
  for(uint8_t i=0;i<8;++i)
  {
      if (sreg & (uint8_t)0x80u)
      {
	nice[i+1]=pgm_read_byte(&nice_pattern[i]);
      }
      else
      {
	nice[i+1]='-';
      }
      sreg = sreg << 1;
  }
  return nice;
}
//-------------------------------------------------------------------------
void VM::instrFetch()
{
  static const char fmt[] PROGMEM = "<%04X>";
  for(uint8_t i=0;i<AVRVIRT_DEBUG_BUFFERSIZE;++i)
  {
    m_dbgLine[i]=' '; 
  }
  snprintf_P(m_dbgLine,AVRVIRT_DEBUG_BUFFERSIZE,fmt,m_pc.word);
  m_dbgLine[93]='-';
  m_dbgLine[94]='-';
  m_dbgLine[95]='>';
}
//-------------------------------------------------------------------------
void VM::instrExecuted()
{
  for(uint8_t i=0;i<(AVRVIRT_DEBUG_BUFFERSIZE-1);++i)
  {
    if (m_dbgLine[i]==0)
      m_dbgLine[i]=' ';
  };
  m_dbgLine[AVRVIRT_DEBUG_BUFFERSIZE-1]=0;
}


} //namespace avrvm

#endif //AVRVIRT_DEBUG