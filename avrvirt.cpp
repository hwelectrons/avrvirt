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

#ifdef AVRVIRT_DEBUG
#	include "avrvirt_debug.h"
#else
#	define DECODED(fmt,...)
#	define BEFORE(fmt,...)
#	define AFTER(fmt,...)
#endif

using namespace avrvirt;
///////////////////////////////////////////////////////////////////////////////
//	VM 
///////////////////////////////////////////////////////////////////////////////

VM::VM()
 : m_Z(*((avr_word_t *)&m_r[30]))
  ,m_Y(*((avr_word_t *)&m_r[28]))
  ,m_X(*((avr_word_t *)&m_r[26]))
{
  m_sreg=0;
  for(uint8_t i=0u;i<32u;++i)
  {
    m_r[i]=0;
  }
  m_pc.word=0;
  m_sp.word=0;
  m_ControlFlags=0;
}
//-------------------------------------------------------------------------    
const uint8_t VM::ReadDataByte(const uint16_t iAddress)
{
  return *((uint8_t*)iAddress);
}
//-------------------------------------------------------------------------    
void VM::WriteDataByte(const uint16_t iAddress,const uint8_t src)
{
  *((uint8_t*)iAddress) = src;
}
//-------------------------------------------------------------------------    
const uint8_t VM::ReadProgramByte(const uint16_t iAddress)
{
  return pgm_read_byte(iAddress);
}
//-------------------------------------------------------------------------    
const uint16_t VM::ReadProgramWord(const uint16_t iAddress)
{
  return pgm_read_word(iAddress << 1U);
}
//-------------------------------------------------------------------------    
const uint8_t VM::ReadDataByte_(const uint16_t iAddress)
{
#ifndef AVRVIRT_FASTMEM
  register avr_word_t address;
  address.word = iAddress;
  if (address.byte.hi == 0)
  {
    if (address.byte.lo < 0x20u)
    {
      return m_r[(uint8_t)iAddress];
    }
    else
    {
      switch(address.byte.lo)
      {
	case 0x5Du:
	  return m_sp.byte.lo;
	  break;
	case 0x5Eu:
	  return m_sp.byte.hi;
	  break;
	case 0x5Fu:
	  return m_sreg;
	  break;
	default:
	  return ReadDataByte(iAddress);
	  break;
      }
    }
  }
  else
  {
#endif
    return ReadDataByte(iAddress);
#ifndef AVRVIRT_FASTMEM
  }
#endif 
}
//-------------------------------------------------------------------------    
void VM::WriteDataByte_(const uint16_t iAddress,const uint8_t src)
{
#ifndef AVRVIRT_FASTMEM
  register avr_word_t address;
  address.word = iAddress;
  if (address.byte.hi == 0)
  {
    if (address.byte.lo < (uint8_t)0x20u)
    {
      m_r[(uint8_t)iAddress] = src;
    }
    else
    {
      switch(address.byte.lo)
      {
	case 0x5Du:
	  m_sp.byte.lo=src;
	  break;
	case 0x5Eu:
	  m_sp.byte.hi=src;
	  break;
	case 0x5Fu:
	  m_sreg=src;
	  break;
	default:
	  WriteDataByte(iAddress,src);
	  break;
      }
    }
  }
  else
  {
#endif    
    WriteDataByte(iAddress,src);
#ifndef AVRVIRT_FASTMEM
  }
#endif  
}
//-------------------------------------------------------------------------    
const uint8_t VM::ReadIOByte_(const uint8_t iAddress)
{
#ifdef AVRVIRT_FASTMEM  
  switch(iAddress)
  {
    case 0x3Du:
      return m_sp.byte.lo;
      break;
    case 0x3Eu:
      return m_sp.byte.hi;
      break;
    case 0x3Fu:
      return m_sreg;
      break;
    default:
      return ReadDataByte(iAddress + 0x20u);
      break;
  }
#else
  return ReadDataByte_(iAddress + 0x20u);
#endif
}
//-------------------------------------------------------------------------    
void VM::WriteIOByte_(const uint8_t iAddress,const uint8_t src)
{
#ifdef AVRVIRT_FASTMEM  
  switch(iAddress)
  {
    case 0x3DU:
      m_sp.byte.lo = src;
      break;
    case 0x3EU:
      m_sp.byte.hi = src;
      break;
    case 0x3FU:
      m_sreg = src;
      break;
    default:
      WriteDataByte(iAddress + 0x20u,src);
      break;
  }
#else  
  WriteDataByte_(iAddress + 0x20u,src);
#endif
}
//-------------------------------------------------------------------------    
void VM::skipNextInstruction()
{
  avr_word_t opcode;
  opcode.word = ReadProgramWord(++m_pc.word);
  if (((opcode.word & 0b1111110000001111u) == 0b1001000000000000u)
     || ((opcode.word & 0b1111111000001100u) == 0b1001010000001100u))
  {
    m_pc.word += 2;
  }
  else
  {
    ++m_pc.word;
  }
  setPCFinished();
}
//-------------------------------------------------------------------------    
void VM::illegalOpcode(const avr_word_t opcode)
{
  DECODED("ILLEGAL: <%#06X>",opcode.word)
  m_ControlFlags |= (uint8_t)eIllegalOpcode | (uint8_t)eAttention;  
}
//-------------------------------------------------------------------------    
void VM::setPCFinished()
{
  m_ControlFlags |= (uint8_t)ePCFinished | (uint8_t)eAttention;
}
//-------------------------------------------------------------------------    
uint8_t& VM::decode_Rr(const avr_word_t opcode)
{
  uint8_t index = opcode.byte.lo & (uint8_t)0x0Fu;
  if (opcode.byte.hi & (uint8_t)0x02u)
  {
    index |= (uint8_t)0x10u;
  }
  return m_r[index];
}
//-------------------------------------------------------------------------    
uint8_t& VM::decode_Rd(const avr_word_t opcode)
{
  uint8_t index = shr4(opcode.byte.lo);
  if (opcode.byte.hi & (uint8_t)0x01u)
  {
    index |= (uint8_t)0x10u;
  }
  return m_r[index];
}
//-------------------------------------------------------------------------    
uint8_t& VM::decode_RdUpper(const avr_word_t opcode)
{
  return m_r[shr4(opcode.byte.lo) + (uint8_t)16U];
}
//-------------------------------------------------------------------------    
uint8_t& VM::decode_RrUpper(const avr_word_t opcode)
{
  return m_r[(opcode.byte.lo & (uint8_t)0x0Fu) + (uint8_t)16U];
}
//-------------------------------------------------------------------------
uint16_t& VM::decode_RdUpper4Pairs(const avr_word_t opcode)
{
  uint8_t iOffset = opcode.byte.lo;
  __asm__ __volatile__
  ("swap %0\n\t"
   "lsl %0\n\t"
   "andi %0,6\n\t" : "+d" (iOffset) :);
  return *((uint16_t *)&m_r[iOffset+24]); 
}
//-------------------------------------------------------------------------
uint8_t VM::decode_K6Bit(const avr_word_t opcode)
{
  return (opcode.byte.lo & (uint8_t)0x0Fu) 
	  | ((opcode.byte.lo & (uint8_t)0xC0u) >> 2); 
}
//-------------------------------------------------------------------------
uint8_t VM::decode_K8Bit(const avr_word_t opcode)
{
  return (opcode.byte.lo & (uint8_t)0x0F) 
	  | shl4(opcode.byte.hi); 
}
//-------------------------------------------------------------------------
avr_word_t& VM::decode_IndexReg(const avr_word_t opcode) 
{
  switch((opcode.byte.lo >> (uint8_t)2u) & (uint8_t)0b11u)
  {
    case 0b00u:
      return m_Z;
      break;
    case 0b10u:
      return m_Y;
      break;
    case 0b11u:
      return m_X;
      break;
  }
}
//-----------------------------------------------------------------------------
void VM::instr_bclr_bset(const avr_word_t opcode)
{ 
  uint8_t iBit = opcode.byte.lo;

  __asm__ __volatile__ 
  ("swap %0\n\t"
   "andi %0,0b00000111\n\t": "+d" (iBit) : );
  
  uint8_t iBitmask = setBit(iBit);

  BEFORE("[SREG=%s]",niceSREG(m_sreg))
  
  if (opcode.byte.lo & (uint8_t)0x80u)
  {
    DECODED("BCLR %u",iBit)
    //bclr
    m_sreg &= ~iBitmask;
  }
  else
  {
    DECODED("BSET %u",iBit)
    //bset
    m_sreg |= iBitmask;
  }
  AFTER("[SREG=%s]",niceSREG(m_sreg))
}
//-----------------------------------------------------------------------------
void VM::instr_bld_bst(const avr_word_t opcode)
{ 
  uint8_t iBit = opcode.byte.lo & (uint8_t)0b00000111u;
  uint8_t& Rd = decode_Rd(opcode);
  uint8_t iBitmask = setBit(iBit);
  
  BEFORE("[SREG=%s, R%u=%#04X]",niceSREG(m_sreg),REGNUM(Rd),Rd);
  
  switch((opcode.byte.hi >> 1) & (uint8_t)0b11u)
  {
  case 0: //bld
    DECODED("BLD R%u,%u",REGNUM(Rd),iBit)
    if (m_sreg & (uint8_t)0x40u)
    {
      Rd |= iBitmask;
    }
    else
    {
      Rd &= ~iBitmask;   
    }
    break;
  case 1: //bst
    DECODED("BST %u,R%u",iBit,REGNUM(Rd))
    if (Rd & iBitmask)
    {
      m_sreg |= (uint8_t)0x40u;
    }
    else
    {
      m_sreg &= ~(uint8_t)0x40u;   
    }
    break;
  case 2: //sbrc
    DECODED("SBRC R%u,%u",REGNUM(Rd),iBit)
    if (!(Rd & iBitmask))
    {
      skipNextInstruction();
    }
    break;
  case 3: //sbrs
    DECODED("SBRS R%u,%u",REGNUM(Rd),iBit)
    if (Rd & iBitmask)
    {
      skipNextInstruction();
    }
    break;
  default: //never reached
    break;
  }   
  
  AFTER("[SREG=%s, R%u=%#04X]",niceSREG(m_sreg),REGNUM(Rd),Rd);
}
//-------------------------------------------------------------------------
void VM::instr_br(const avr_word_t opcode)
{
  uint8_t iBit = opcode.byte.lo & (uint8_t)0b00000111u;
  int8_t k = ((int8_t)(opcode.word >> 2)) / 2u;
  int8_t iOffsetA;
  int8_t iOffsetB;
  BEFORE("[PC=%#06X]",m_pc.word)
  
  if (opcode.byte.hi & (uint8_t)0b00000100u)
  {
    DECODED("BRBC %u,%d",iBit,k)
    iOffsetA = 0;
    iOffsetB = k;
  }
  else
  {
    DECODED("BRBS %u,%d",iBit,k)
    iOffsetA = k;
    iOffsetB = 0;
  }
  
  if (m_sreg & setBit(iBit))
  {
    m_pc.word += iOffsetA+1;
  }
  else
  {
    m_pc.word += iOffsetB+1;    
  }
  setPCFinished();
  AFTER("[PC=%#06X]",m_pc.word)
}
//-------------------------------------------------------------------------
void VM::instr_call_jmp(const avr_word_t opcode)
{
  register avr_word_t opcode2;
  register avr_word_t pc;
  BEFORE("[PC=%#06X, SP=%#06X]",m_pc.word,m_sp.word)
  pc.word = m_pc.word + 1; 
  opcode2.word = ReadProgramWord(pc.word);
  if (opcode.byte.lo & (uint8_t)0b10u) //call flag
  {
    DECODED("CALL %#06X",opcode2.word);
    ++pc.word;
    WriteDataByte_(--m_sp.word,pc.byte.hi);
    WriteDataByte_(--m_sp.word,pc.byte.lo);
  }
#ifdef AVRVIRT_DEBUG
  else
  {
    DECODED("JMP %#06X",opcode2.word);
  }
#endif
  m_pc.word = opcode2.word;
  AFTER("[PC=%#06X, SP=%#06X]",m_pc.word,m_sp.word)
  setPCFinished();
}
//-----------------------------------------------------------------------------
void VM::instr_cbi_sbi(const avr_word_t opcode)
{
  uint8_t iAddress = (opcode.byte.lo >> 3u);
  uint8_t iData = ReadIOByte_(iAddress);
  uint8_t iMask = setBit(opcode.byte.lo & (uint8_t)0x07u);
#ifdef AVRVIRT_DEBUG
  uint8_t iBit = opcode.byte.lo & (uint8_t)0x07u;
  uint8_t iIO = iAddress;
#endif
  BEFORE("[(%#04X)=%#04X, PC=%#06X]",iIO,iData,m_pc.word)
  switch(opcode.byte.hi & (uint8_t) 0b11u)
  {
    case 0b00u: //cbi
      DECODED("CBI %u,%u",iIO,iBit)
      iData &= ~(iMask);
      WriteIOByte_(iAddress,iData);
      break;
    case 0b01u: //sbic
      DECODED("SBIC %u,%u",iIO,iBit)
      if (!(iData & iMask))
      {
	 skipNextInstruction();
      }
      break;
    case 0b10u: //sbi
      DECODED("SBI %u,%u",iIO,iBit)
      iData |= iMask;
      WriteIOByte_(iAddress,iData);
      break;
    case 0b11u: //sbis
      DECODED("SBIS %u,%u",iIO,iBit)
      if (iData & iMask)
      {
	 skipNextInstruction();
      }
  }
  AFTER("[(%#04X)=%#04X, PC=%#06X]",iIO,iData,m_pc.word)
}
//-----------------------------------------------------------------------------
void VM::instr_reg_reg_im8(const avr_word_t opcode, const uint8_t subcode)
{
  register uint8_t Rr = (subcode & (uint8_t)0x10u) ? decode_K8Bit(opcode) : decode_Rr(opcode); 
  uint8_t& Rd = (subcode & (uint8_t)0x10u) ? decode_RdUpper(opcode) : decode_Rd(opcode);
  register uint8_t sreg_mask = 0;
  register uint8_t tmp_sreg = m_sreg;
  register uint8_t Rd_ = Rd;
#ifdef AVRVIRT_DEBUG
  uint8_t RdIndex; 
  uint8_t RrIndex;
  RdIndex = getRdIndex(opcode);
  if (subcode & (uint8_t)0x10u)
  {
    BEFORE("[R%u=%#04X, SREG=%s]",RdIndex,Rd,niceSREG(m_sreg))
  }
  else
  {
    RrIndex = getRrIndex(opcode);
    BEFORE("[R%u=%#04X, R%u=%#04X, SREG=%s]",RdIndex,Rd,RrIndex,Rr,niceSREG(m_sreg))
  }
#endif    
  switch(subcode & (uint8_t)0x0Fu)
  {
    case 0: //adc
      DECODED("ADC R%u,R%u",RdIndex,RrIndex)
        if (m_sreg & 1)
	{
	  __asm__ __volatile__ 
	  ("sec\n\t"
	   "adc %0,%2\n\t"
	   "in %1,__SREG__\n\t": "+r" (Rd_),"=&r" (tmp_sreg) : "r" (Rr));
	}
	else
	{
	  __asm__ __volatile__ 
	  ("clc\n\t"
	   "adc %0,%2\n\t"
	   "in %1,__SREG__\n\t": "+r" (Rd_),"=&r" (tmp_sreg) : "r" (Rr));	
	}
	sreg_mask = 0b00111111u;
	break;
    case 1: //add
      DECODED("ADD R%u,R%u",RdIndex,RrIndex)
        __asm__ __volatile__ 
	("add %0,%2\n\t"
	 "in %1,__SREG__\n\t": "+r" (Rd_),"=&r" (tmp_sreg) : "r" (Rr));
	sreg_mask = 0b00111111u;
	break;
    case 2: //and, andi
#ifdef AVRVIRT_DEBUG
	if (subcode & (uint8_t)0x10u)
	  DECODED("ANDI R%u,%u",RdIndex,Rr)
	else
	  DECODED("AND R%u,R%u",RdIndex,RrIndex)
#endif
        __asm__ __volatile__ 
	("and %0,%2\n\t"
	 "in %1,__SREG__\n\t": "+r" (Rd_),"=&r" (tmp_sreg) : "r" (Rr));
	sreg_mask = 0b00011110u;
	break;
    case 3: //cp, cpi
#ifdef AVRVIRT_DEBUG
	if (subcode & (uint8_t)0x10u)
	  DECODED("CPI R%u,%u",RdIndex,Rr)
	else
	  DECODED("CP R%u,R%u",RdIndex,RrIndex)
#endif
        __asm__ __volatile__ 
	("cp %1,%2\n\t"
	 "in %0,__SREG__\n\t": "=&r" (tmp_sreg) : "r" (Rd_),"r" (Rr));
	sreg_mask = 0b00111111u;
	break;
    case 4: //cpc
	DECODED("CPC R%u,R%u",RdIndex,RrIndex)
	__asm__ __volatile__ 
	(
	  "andi %0,0b11\n\t"
	  "in __tmp_reg__,__SREG__\n\t"
	  "out __SREG__,%0\n\t"
	  "cpc %1,%2\n\t"
	  "in %0,__SREG__\n\t"
	  "out __SREG__,__tmp_reg__\n\t"
	  : "+d" (tmp_sreg) : "r" (Rd_),"r" (Rr)
	);
	sreg_mask = 0b00111111u;
      break;
    case 5: //eor
	  DECODED("EOR R%u,R%u",RdIndex,RrIndex)
        __asm__ __volatile__ 
	("eor %0,%2\n\t"
	 "in %1,__SREG__\n\t": "+r" (Rd_),"=&r" (tmp_sreg) : "r" (Rr));
	sreg_mask = 0b00011110u;
	break;
    case 6: //mov
      DECODED("MOV R%u,R%u",RdIndex,RrIndex)
	Rd_ = Rr;
	break;
    case 7: //or, ori
#ifdef AVRVIRT_DEBUG
	if (subcode & (uint8_t)0x10u)
	  DECODED("ORI R%u,%u",RdIndex,Rr)
	else
	  DECODED("OR R%u,R%u",RdIndex,RrIndex)
#endif
        __asm__ __volatile__ 
	("or %0,%2\n\t"
	 "in %1,__SREG__\n\t": "+r" (Rd_),"=&r" (tmp_sreg) : "r" (Rr));
	sreg_mask = 0b00011110u;
	break;
    case 8: //sbc, sbci
#ifdef AVRVIRT_DEBUG
	if (subcode & (uint8_t)0x10u)
	  DECODED("SBCI R%u,%u",RdIndex,Rr)
	else
	  DECODED("SBC R%u,R%u",RdIndex,RrIndex)
#endif
	__asm__ __volatile__ 
	(
	  "andi %0,0b11\n\t"
	  "in __tmp_reg__,__SREG__\n\t"
	  "out __SREG__,%0\n\t"
	  "sbc %1,%2\n\t"
	  "in %0,__SREG__\n\t"
	  "out __SREG__,__tmp_reg__\n\t"
	  : "+d" (tmp_sreg) : "r" (Rd_),"r" (Rr)
	);
	sreg_mask = 0b001111111;
	break;
    case 9: //sub, subi
#ifdef AVRVIRT_DEBUG
	if (subcode & (uint8_t)0x10u)
	  DECODED("SUBI R%u,%u",RdIndex,Rr)
	else
	  DECODED("SUB R%u,R%u",RdIndex,RrIndex)
#endif
        __asm__ __volatile__ 
	("sub %0,%2\n\t"
	 "in %1,__SREG__\n\t": "+r" (Rd_),"=&r" (tmp_sreg) : "r" (Rr));
	sreg_mask = 0b00111111u;
	break;
    default:
	illegalOpcode(opcode);
	break;
  }
  m_sreg = (m_sreg & ~sreg_mask) | (tmp_sreg & sreg_mask);
  Rd = Rd_;
#ifdef AVRVIRT_DEBUG
  if (subcode & (uint8_t)0x10u)
  {
    AFTER("[R%u=%#04X, SREG=%s]",RdIndex,Rd,niceSREG(m_sreg))
  }
  else
  {
    AFTER("[R%u=%#04X, R%u=%#04X, SREG=%s]",RdIndex,Rd,RrIndex,Rr,niceSREG(m_sreg))
  }
#endif    
  
}
//-----------------------------------------------------------------------------
void VM::instr_cpse(const avr_word_t opcode)
{
  uint8_t Rd = decode_Rd(opcode);
  uint8_t Rr = decode_Rr(opcode);
#ifdef AVRVIRT_DEBUG
  uint8_t RdIndex = getRdIndex(opcode);
  uint8_t RrIndex = getRdIndex(opcode);
  BEFORE("[R%u=%#04X, R%u=%#04X, PC=%#06X]",RdIndex,Rd,RrIndex,Rr,m_pc.word)
  DECODED("CPSE R%u,R%u",RdIndex,RrIndex)
#endif
  if (Rd == Rr)
    skipNextInstruction();
  AFTER("[R%u=%#04X, R%u=%#04X, PC=%#06X]",RdIndex,Rd,RrIndex,Rr,m_pc.word)
}
//-----------------------------------------------------------------------------
void VM::instr_16im(const avr_word_t opcode)
{
  uint16_t& Rd = decode_RdUpper4Pairs(opcode);
  uint8_t K = decode_K6Bit(opcode);
  uint8_t tmp_sreg;
#ifdef AVRVIRT_DEBUG
  uint8_t RdUpperIndex = getRdUpper4PairsIndex(opcode);
  BEFORE("[R%u:%u=%#06X, SREG=%s]",RdUpperIndex+1,RdUpperIndex,Rd,niceSREG(m_sreg))
#endif
  if (opcode.byte.hi & (uint8_t)0b1u) //sbiw
  {
    DECODED("SBIW R%u,%u",RdUpperIndex,K)
    __asm__ __volatile__ 
    ("sub %A0,%2\n\t"
     "sbc %B0,__zero_reg__\n\t"
     "in %1,__SREG__\n\t": "+r" (Rd),"=&r" (tmp_sreg) : "r" (K));
  }
  else //adiw
  {
    DECODED("ADIW R%u,%u",RdUpperIndex,K)
    __asm__ __volatile__ 
    ("add %A0,%2\n\t"
     "adc %B0,__zero_reg__\n\t"
     "in %1,__SREG__\n\t": "+r" (Rd),"=&r" (tmp_sreg) : "r" (K));
  }
  m_sreg = (m_sreg & (uint8_t)0b11100000u) | (tmp_sreg & (uint8_t)0b00011101u);
  m_sreg |= (Rd == 0) ? (uint8_t)0b10u : (uint8_t)0u; 
  AFTER("[R%u:%u=%#06X, SREG=%s]",RdUpperIndex+1,RdUpperIndex,Rd,niceSREG(m_sreg)) 
}
//-----------------------------------------------------------------------------
void VM::instr_reg(const avr_word_t opcode)
{
  uint8_t& Rd = decode_Rd(opcode);
  uint8_t tmp_sreg = 0;
  uint8_t sreg_mask = 0;
  #ifdef AVRVIRT_DEBUG
  uint8_t RdIndex = getRdIndex(opcode);
  BEFORE("[R%u=%#04X, SREG=%s]",RdIndex,Rd,niceSREG(m_sreg))
  #endif
  if (opcode.byte.lo & 0b1000u) //dec: 0b1010u - because of upper level decoding 
				//it's enough to test first bit
  {
    DECODED("DEC R%u",RdIndex)
    __asm__ __volatile__ 
    ("dec %0\n\t"
    "in %1,__SREG__\n\t" : "+r" (Rd),"=&r" (tmp_sreg) : );
    sreg_mask = 0b00011110u;
  }
  else
  {
    switch(opcode.byte.lo & (uint8_t)0x07u)
    {
      case 0b000u: //com
	DECODED("COM R%u",RdIndex)
	__asm__ __volatile__ 
	("com %0\n\t"
	"in %1,__SREG__\n\t" : "+r" (Rd),"=&r" (tmp_sreg) : );
	sreg_mask = 0b00011111u;
	break;
      case 0b001u: //neg
	DECODED("NEG R%u",RdIndex)
	__asm__ __volatile__ 
	("neg %0\n\t"
	"in %1,__SREG__\n\t" : "+r" (Rd),"=&r" (tmp_sreg) : );
	sreg_mask = 0b00111111u;
	break;
      case 0b010u: //swap
	DECODED("SWAP R%u",RdIndex)
	__asm__ __volatile__ 
	("swap %0\n\t" : "+r" (Rd) : );
	break;
      case 0b011u: //inc
	DECODED("INC R%u",RdIndex)
	__asm__ __volatile__ 
	("inc %0\n\t"
	"in %1,__SREG__\n\t" : "+r" (Rd),"=&r" (tmp_sreg) : );
	sreg_mask = 0b00011110u;
	break;
      case 0b100u:
	illegalOpcode(opcode);
	break;
      case 0b101u: //asr
	DECODED("ASR R%u",RdIndex)
	__asm__ __volatile__ 
	("asr %0\n\t"
	"in %1,__SREG__\n\t" : "+r" (Rd),"=&r" (tmp_sreg) : );
	sreg_mask = 0b00011111u;
	break;
      case 0b110u: //lsr
	DECODED("LSR R%u",RdIndex)
	__asm__ __volatile__ 
	("lsr %0\n\t"
	"in %1,__SREG__\n\t" : "+r" (Rd),"=&r" (tmp_sreg) : );
	sreg_mask = 0b00011111u;
	break;
      case 0b111u: //ror
	DECODED("ROR R%u",RdIndex)
	if (m_sreg & 1)
	{
	  __asm__ __volatile__ 
	  ("sec\n\t"
	  "ror %0\n\t"
	  "in %1,__SREG__\n\t": "+r" (Rd),"=&r" (tmp_sreg) : );
	}
	else
	{
	  __asm__ __volatile__ 
	  ("clc\n\t"
	  "ror %0\n\t"
	  "in %1,__SREG__\n\t": "+r" (Rd),"=&r" (tmp_sreg) : );	
	}
	sreg_mask = 0b00011111u;
	break;
    }
  }
  m_sreg = (m_sreg & ~sreg_mask) | (tmp_sreg & sreg_mask);
  AFTER("[R%u=%#04X, SREG=%s]",RdIndex,Rd,niceSREG(m_sreg))
}
//-----------------------------------------------------------------------------
void VM::instr_mul(const avr_word_t opcode)
{
#ifdef __AVR_HAVE_MUL__  
  uint8_t& Rd = decode_Rd(opcode);
  uint8_t& Rr = decode_Rr(opcode);
  volatile uint8_t *ptr_r0 = &m_r[0];
//  uint16_t iResult;
  uint8_t tmp_sreg;
#ifdef AVRVIRT_DEBUG
  uint16_t *r0 = (uint16_t*)&m_r[0];
  uint8_t RdIndex = getRdIndex(opcode);
  uint8_t RrIndex = getRrIndex(opcode);
  BEFORE("[R1:R0=%#06X, R%u=%#04X, R%u=%#04X, SREG=%s]",*r0,RdIndex,Rd,RrIndex,Rr,niceSREG(m_sreg))
#endif
  DECODED("MUL R%u,R%u",RdIndex,RrIndex)
  __asm__ __volatile__ 
  ("push r0\n\t"
   "push r1\n\t"
   "mul %1,%2\n\t"
   "in %0,__SREG__\n\t"
   "st %a3+,r0\n\t"
   "st %a3,r1\n\t"
   "pop r1\n\t"
   "pop r0\n\t"
   : "=&r" (tmp_sreg) : "r" (Rd), "r" (Rr), "e" (ptr_r0) : "r0","r1");
/*
  __asm__ __volatile__ 
  ("push r0\n\t"
   "push r1\n\t"
   "mul %2,%3\n\t"
   "in %0,__SREG__\n\t"
   "movw %A1,r0\n\t"
   "pop r1\n\t"
   "pop r0\n\t"
   : "=&r" (tmp_sreg), "=w"(iResult) : "r" (Rd), "r" (Rr) : "r0","r1");
   *(uint16_t*)&m_r[0]=iResult;*/
  m_sreg = (m_sreg & (uint8_t)0b11111100u) | (tmp_sreg & (uint8_t)0b00000011u);
  AFTER("[R1:R0=%#06X, R%u=%#04X, R%u=%#04X, SREG=%s]",*r0,RdIndex,Rd,RrIndex,Rr,niceSREG(m_sreg))
#else
  illegalOpcode(opcode);
#endif
}
//-----------------------------------------------------------------------------   
void VM::instr_muls(const avr_word_t opcode)
{
#ifdef __AVR_HAVE_MUL__  
  uint8_t& Rd = decode_RdUpper(opcode);
  uint8_t& Rr = decode_RrUpper(opcode);
  volatile uint8_t *ptr_r0 = &m_r[0];
  uint8_t tmp_sreg;
#ifdef AVRVIRT_DEBUG
  uint16_t *r0 = (uint16_t*)ptr_r0;
  uint8_t RdIndex = getRdUpperIndex(opcode);
  uint8_t RrIndex = getRrUpperIndex(opcode);
  BEFORE("[R1:R0=%#06X, R%u=%#04X, R%u=%#04X, SREG=%s]",*r0,RdIndex,Rd,RrIndex,Rr,niceSREG(m_sreg))
#endif
  DECODED("MULS R%u,R%u",RdIndex,RrIndex)
  __asm__ __volatile__ 
  ("push r0\n\t"
   "push r1\n\t"
   "muls %1,%2\n\t"
   "in %0,__SREG__\n\t"
   "st %a3+,r0\n\t"
   "st %a3,r1\n\t" 
   "pop r1\n\t"
   "pop r0\n\t" : "=&r" (tmp_sreg) : "d" (Rd), "d" (Rr), "e" (ptr_r0) : "r0","r1");
  m_sreg = (m_sreg & (uint8_t)0b11111100u) | (tmp_sreg & (uint8_t)0b00000011u);
  AFTER("[R1:R0=%#06X, R%u=%#04X, R%u=%#04X, SREG=%s]",*r0,RdIndex,Rd,RrIndex,Rr,niceSREG(m_sreg))
#else
  illegalOpcode(opcode);
#endif
}
//-----------------------------------------------------------------------------   
void VM::instr_fmul(const avr_word_t opcode)
{
#ifdef __AVR_HAVE_MUL__  
  uint8_t& Rd = m_r[16 + (shr4(opcode.byte.lo) & (uint8_t)0b111u)];
  uint8_t& Rr = m_r[16 + (opcode.byte.lo & (uint8_t)0b111u)];
  volatile uint8_t *ptr_r0 = &m_r[0];
  uint8_t tmp_sreg;
#ifdef AVRVIRT_DEBUG
  uint16_t *r0 = (uint16_t*)ptr_r0;
  uint8_t RdIndex = 16 + (shr4(opcode.byte.lo) & (uint8_t)0b111u);
  uint8_t RrIndex = 16 + (opcode.byte.lo & (uint8_t)0b111u);
  BEFORE("[R1:R0=%#06X, R%u=%#04X, R%u=%#04X, SREG=%s]",*r0,RdIndex,Rd,RrIndex,Rr,niceSREG(m_sreg))
#endif
  if (opcode.byte.lo & (uint8_t)0x80u)
  {
    if (opcode.byte.lo & (uint8_t)0x08u) //fmulsu
    {
      DECODED("FMULSU R%u,R%u",RdIndex,RrIndex)
      __asm__ __volatile__ 
      ("push r0\n\t"
       "push r1\n\t"
       "fmulsu %1,%2\n\t"
       "in %0,__SREG__\n\t"
       "st %a3+,r0\n\t"
       "st %a3,r1\n\t"
       "pop r1\n\t"
       "pop r0\n\t" : "=&r" (tmp_sreg) : "a" (Rd), "a" (Rr), "e" (ptr_r0) : "r0","r1");
    }
    else //fmuls
    {
      DECODED("FMULS R%u,R%u",RdIndex,RrIndex)
      __asm__ __volatile__ 
      ("push r0\n\t"
       "push r1\n\t"
       "fmuls %1,%2\n\t"
       "in %0,__SREG__\n\t"
       "st %a3+,r0\n\t"
       "st %a3,r1\n\t"
       "pop r1\n\t"
       "pop r0\n\t"  : "=&r" (tmp_sreg) : "a" (Rd), "a" (Rr), "e" (ptr_r0) : "r0","r1");      
    }
  }
  else
  {
    if (opcode.byte.lo & (uint8_t)0x08u) //fmul
    {
      DECODED("FMUL R%u,R%u",RdIndex,RrIndex)
      __asm__ __volatile__ 
      ("push r0\n\t"
       "push r1\n\t"
       "fmul %1,%2\n\t"
       "in %0,__SREG__\n\t"
       "st %a3+,r0\n\t"
       "st %a3,r1\n\t"
       "pop r1\n\t"
       "pop r0\n\t"  : "=&r" (tmp_sreg) : "a" (Rd), "a" (Rr), "e" (ptr_r0) : "r0","r1");
    }
    else //mulsu
    {
      DECODED("MULSU R%u,R%u",RdIndex,RrIndex)
      __asm__ __volatile__ 
      ("push r0\n\t"
       "push r1\n\t"
       "mulsu %1,%2\n\t"
       "in %0,__SREG__\n\t"
       "st %a3+,r0\n\t"
       "st %a3,r1\n\t"
       "pop r1\n\t"
       "pop r0\n\t"   : "=&r" (tmp_sreg) : "a" (Rd), "a" (Rr), "e" (ptr_r0) : "r0","r1");      
    }
  }
  m_sreg = (m_sreg & (uint8_t)0b11111100u) | (tmp_sreg & (uint8_t)0b00000011u);
  AFTER("[R1:R0=%#06X, R%u=%#04X, R%u=%#04X, SREG=%s]",*r0,RdIndex,Rd,RrIndex,Rr,niceSREG(m_sreg))
#else
  illegalOpcode(opcode);
#endif 
}
//-----------------------------------------------------------------------------   
void VM::instr_icall_ijmp(const avr_word_t opcode)
{
  BEFORE("[R31:R30=%#06X, SP=%#06X, PC=%#06X",m_Z.word,m_sp.word,m_pc.word)
  if (opcode.byte.hi & (uint8_t)0x01u) //icall
  {
    DECODED("%s","ICALL")
    register avr_word_t pc;
    pc.word = m_pc.word + 1;
    WriteDataByte_(--m_sp.word,pc.byte.hi);
    WriteDataByte_(--m_sp.word,pc.byte.lo);
  }
#ifdef AVRVIRT_DEBUG  
  else
  {
    DECODED("%s","IJMP")
  }
#endif  
  m_pc.word = m_Z.word;
  setPCFinished();
  AFTER("[R31:R30=%#06X, SP=%#06X, PC=%#06X",m_Z.word,m_sp.word,m_pc.word)
}
//-----------------------------------------------------------------------------   
void VM::instr_in_out(const avr_word_t opcode)	
{
  uint8_t& r = decode_Rd(opcode);
  uint8_t A = ((opcode.byte.hi & (uint8_t)0b110u) << 3) 
	      | (opcode.byte.lo & (uint8_t)0b1111u);

#ifdef AVRVIRT_DEBUG  
  uint8_t rIndex = getRdIndex(opcode);
  BEFORE("[R%u=%#04X]",rIndex,r)
#endif 
  
  if (opcode.byte.hi & (uint8_t)0b1000u) //out
  {
    DECODED("OUT %#04X,R%u",A,rIndex)
    WriteIOByte_(A,r);
  } //in
  else
  {
    DECODED("IN R%u,%#04X",rIndex,A)
    r=ReadIOByte_(A);
  }
  AFTER("[R%u=%#04X]",rIndex,r)
}
//-----------------------------------------------------------------------------   
void VM::instr_pop_push(const avr_word_t opcode)
{
  uint8_t& Rd = decode_Rd(opcode); 
#ifdef AVRVIRT_DEBUG  
  uint8_t rIndex = getRdIndex(opcode);
  BEFORE("[R%u=%#04X, SP=%#06X]",rIndex,Rd,m_sp.word)
#endif
  if (opcode.byte.hi & (uint8_t)0b10u) //push
  {
    DECODED("PUSH R%u",rIndex)
    WriteDataByte_(--m_sp.word,Rd);
  }
  else //pop
  {
    DECODED("POP R%u",rIndex)
    Rd = ReadDataByte_(m_sp.word++);
  }
  AFTER("[R%u=%#04X, SP=%#06X]",rIndex,Rd,m_sp.word)
}
//-----------------------------------------------------------------------------   
void VM::instr_ld_st(const avr_word_t opcode)
{
  uint8_t& r = decode_Rd(opcode);
  avr_word_t& IndexReg = decode_IndexReg(opcode);
#ifdef AVRVIRT_DEBUG
  uint8_t rIndex = getIndexRegIndex(opcode);
  uint8_t RdIndex = getRdIndex(opcode);
  BEFORE("[R%u:R%u=%#06X, R%u=%#04X]",rIndex+1,rIndex,IndexReg.word,RdIndex,r)
  if (opcode.byte.hi & (uint8_t)0b10u) //st
  {
    if ((opcode.byte.lo & 0b1111u) == 0b1100u)
    {
      DECODED("ST X,R%u",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b1101u)
    {
      DECODED("ST X+,R%u",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b1110u)
    {
      DECODED("ST -X,R%u",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b1000u)
    {
      DECODED("ST Y,R%u",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b1001u)
    {
      DECODED("ST Y+,R%u",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b1010u)
    {
      DECODED("ST -Y,R%u",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b0000u)
    {
      DECODED("ST Z,R%u",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b0001u)
    {
      DECODED("ST Z+,R%u",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b0010u)
    {
      DECODED("ST -Z,R%u",RdIndex)
    }
  }else
  {
    if ((opcode.byte.lo & 0b1111u) == 0b1100u)
    {
      DECODED("LD R%u,X",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b1101u)
    {
      DECODED("LD R%u,X+",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b1110u)
    {
      DECODED("LD R%u,-X",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b1000u)
    {
      DECODED("LD R%u,Y",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b1001u)
    {
      DECODED("LD R%u,Y+",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b1010u)
    {
      DECODED("LD R%u,-Y",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b0000u)
    {
      DECODED("LD R%u,Z",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b0001u)
    {
      DECODED("LD R%u,Z+",RdIndex)
    } else if ((opcode.byte.lo & 0b1111u) == 0b0010u)
    {
      DECODED("LD R%u,-Z",RdIndex)
    }
  }
  #endif
  
  if (opcode.byte.lo & (uint8_t)0b10u)
  {
    --IndexReg.word;
  }
  if (opcode.byte.hi & (uint8_t)0b10u) //st
  {
    WriteDataByte_(IndexReg.word,r);
  }
  else //ld
  {
    r = ReadDataByte_(IndexReg.word);   
  }
  if (opcode.byte.lo & (uint8_t)0b01u)
  {
    ++IndexReg.word;
  }
  
  AFTER("[R%u:R%u=%#06X, R%u=%#04X]",rIndex+1,rIndex,IndexReg.word,RdIndex,r)
}
//-----------------------------------------------------------------------------   
void VM::instr_ldd_std(const avr_word_t opcode)
{
  uint8_t& r = decode_Rd(opcode);
  uint8_t q = opcode.byte.lo;
  uint16_t iAddress = (opcode.byte.lo & (uint8_t)0b1000u) ? m_Y.word : m_Z.word;

  __asm__ __volatile__ 
  ( "andi %0,0b111\n\t"
    "bst %1,2\n\t"
    "bld %0,3\n\r"
    "bst %1,3\n\t"
    "bld %0,4\n\t"
    "bst %1,5\n\t"
    "bld %0,5\n\t"
    : "+d" (q) : "r" (opcode.byte.hi) );
  
  
#ifdef AVRVIRT_DEBUG
  uint8_t rIndex = (opcode.byte.lo & (uint8_t)0b1000u) ? 28 : 30;
  uint8_t RdIndex = getRdIndex(opcode);
  BEFORE("[R%u:R%u=%#06X, R%u=%#04X]",rIndex+1,rIndex,iAddress,RdIndex,r)
  if ((opcode.word & 0b0000001000001000u) == 0b0000001000001000u)
  {
    DECODED("STD Y+%u,R%u",q,RdIndex);
  }else if ((opcode.word & 0b0000001000001000u) == 0b0000001000000000u)
  {
    DECODED("STD Z+%u,R%u",q,RdIndex);
  }else if ((opcode.word & 0b0000001000001000u) == 0b0000000000001000u)
  {
    DECODED("LDD R%u,Y+%u",RdIndex,q);
  }else if ((opcode.word & 0b0000001000001000u) == 0b0000000000000000u)
  {
    DECODED("LDD R%u,Z+%u",RdIndex,q);
  }
#endif
  
  iAddress+=q;
  if (opcode.byte.hi & (uint8_t)0b10u) //std
  {
    WriteDataByte_(iAddress,r);
  }
  else //ldd
  {
    r = ReadDataByte_(iAddress);
  }
  
  AFTER("[R%u:R%u=%#06X, R%u=%#04X]",rIndex+1,rIndex,iAddress-q,RdIndex,r)
}
//-----------------------------------------------------------------------------   
void VM::instr_ldi(const avr_word_t opcode)
{
  uint8_t& r = decode_RdUpper(opcode);
#ifdef AVRVIRT_DEBUG
  uint8_t RdIndex = getRdUpperIndex(opcode);
  BEFORE("[R%u=%#04X]",RdIndex,r)
#endif
  DECODED("LDI R%u,%#04X",RdIndex,shl4(opcode.byte.hi) | (opcode.byte.lo & (uint8_t)0x0Fu))
  r = shl4(opcode.byte.hi) | (opcode.byte.lo & (uint8_t)0x0Fu);
  AFTER("[R%u=%#04X]",RdIndex,r) 
}
//-----------------------------------------------------------------------------   
void VM::instr_lds_sts(const avr_word_t opcode) 
{
  uint16_t iAddress = ReadProgramWord(++m_pc.word);
  uint8_t& Rd = decode_Rd(opcode);
#ifdef AVRVIRT_DEBUG
  uint8_t RdIndex = getRdIndex(opcode);
  BEFORE("[R%u=%#04X]",RdIndex,Rd)
#endif
  if (opcode.byte.hi & (uint8_t)0b10u) //sts
  {
    DECODED("STS %#06X,R%u",iAddress,RdIndex)
    WriteDataByte_(iAddress,Rd);
  }
  else //lds
  {
    DECODED("LDS R%u,%#06X",RdIndex,iAddress)
    Rd = ReadDataByte_(iAddress);
  }
  AFTER("[R%u=%#04X]",RdIndex,Rd)
}
//-----------------------------------------------------------------------------   
void VM::instr_lds7_sts7(const avr_word_t opcode)
{
  uint8_t& Rd = decode_RdUpper(opcode);
  uint8_t iAddress = shl4(opcode.byte.hi & (uint8_t)0b111u) | (opcode.byte.lo & (uint8_t)0b1111u);
  if (opcode.byte.hi & (uint8_t)0b1000u) //sts
  {
    WriteDataByte_(iAddress,Rd);
  }
  else //lds
  {
    Rd = ReadDataByte_(iAddress);
  }
}
//-----------------------------------------------------------------------------   
void VM::instr_lpm(const avr_word_t opcode)
{
  uint8_t& Rd = decode_Rd(opcode);
#ifdef AVRVIRT_DEBUG
  uint8_t RdIndex = getRdIndex(opcode);
  BEFORE("[R%u=%#04X]",RdIndex,Rd)
  if (opcode.byte.lo & (uint8_t)1u)
  {
    DECODED("LPM R%u,Z+",RdIndex)
  }
  else
  {
    DECODED("LPM R%u,Z",RdIndex)
  }
#endif
  Rd = ReadProgramByte((opcode.byte.lo & (uint8_t)1u) ? m_Z.word++ : m_Z.word);
  AFTER("[R%u=%#04X]",RdIndex,Rd)
}
//-----------------------------------------------------------------------------
void VM::instr_movw(const avr_word_t opcode)
{
  uint16_t& Rd = *((uint16_t *)&m_r[shr4(opcode.byte.lo) << 1]);
  uint16_t& Rr = *((uint16_t *)&m_r[(opcode.byte.lo & (uint8_t)0xFu) << 1]);
#ifdef AVRVIRT_DEBUG
  uint8_t RdIndex = shr4(opcode.byte.lo) << 1;
  uint8_t RrIndex = (opcode.byte.lo & (uint8_t)0xFu) << 1;
  BEFORE("[R%u:R%u=%#06X, R%u:R%u=%#06X]",RdIndex+1,RdIndex,Rd,RrIndex+1,RrIndex,Rr)
  DECODED("MOVW R%u,R%u",RdIndex,RrIndex);
#endif
  Rd = Rr;
  AFTER("[R%u:R%u=%#06X, R%u:R%u=%#06X]",RdIndex+1,RdIndex,Rd,RrIndex+1,RrIndex,Rr)
}
//-----------------------------------------------------------------------------
void VM::instr_rcall_rjmp(const avr_word_t opcode)
{
  avr_word_t k;
  k.word = opcode.word;
  if (k.byte.hi & 0x08u) //signed
  {
    k.byte.hi |= (uint8_t)0xF0u;
  }
  else
  {
     k.byte.hi &= (uint8_t)0x0Fu;
  }
  BEFORE("[PC=%#06X, SP=%#06X]",m_pc.word,m_sp.word)
  ++m_pc.word;
  if (opcode.byte.hi & (uint8_t)0b10000u) //call
  {
    DECODED("RCALL %i",(int16_t)k.word)
    WriteDataByte_(--m_sp.word,m_pc.byte.hi);
    WriteDataByte_(--m_sp.word,m_pc.byte.lo);
  }
#ifdef AVRVIRT_DEBUG  
  else
  {
    DECODED("RJMP %i",(int16_t)k.word)
  }
#endif
  m_pc.word+=k.word;
  setPCFinished();
  AFTER("[PC=%#06X, SP=%#06X]",m_pc.word,m_sp.word)
}
//-----------------------------------------------------------------------------
void VM::instr_misc(const avr_word_t opcode)
{
  uint8_t subcode = shr4(opcode.byte.lo);
  switch(subcode)
  {
      case 0b0000u: //ret
      case 0b0001u: //reti
	BEFORE("[PC=%#06X, SP=%#06X]",m_pc.word,m_sp.word)
	if (subcode & (uint8_t)1u) //reti
	{
	  m_sreg |= (uint8_t)0x80u;
	  DECODED("%s","RETI")
	}
#ifdef AVRVIRT_DEBUG
	else
	{
 	  DECODED("%s","RET")
	}
#endif	
	m_pc.byte.lo = ReadDataByte_(m_sp.word++);
	m_pc.byte.hi = ReadDataByte_(m_sp.word++);
	setPCFinished();
	AFTER("[PC=%#06X, SP=%#06X]",m_pc.word,m_sp.word)
	break;
      case 0b1000u: //sleep
	DECODED("%s","SLEEP")
	m_ControlFlags |= ((uint8_t)eAttention | (uint8_t)eSleepOpcode);
	break;
      case 0b1001u: //break
	DECODED("%s","BREAK")
	m_ControlFlags |= ((uint8_t)eAttention | (uint8_t)eBreakOpcode);
	break;
      case 0b1010u: //wdr
	DECODED("%s","WDR")
	m_ControlFlags |= ((uint8_t)eAttention | (uint8_t)eWDROpcode);
	break;
      case 0b1100u: //lpm
	BEFORE("[R0=%#04X, Z=%#06X]",m_r[0],m_Z.word)
	DECODED("%s","LPM")
	m_r[0] = ReadProgramByte(m_Z.word);
	AFTER("[R0=%#04X, Z=%#06X]",m_r[0],m_Z.word)
	break;
      default:
	illegalOpcode(opcode);
	break;
  }
}
//-----------------------------------------------------------------------------
uint8_t VM::run(const uint8_t iMaximumNumberOfInstructions)
{
  avr_word_t iCounter;
  avr_word_t opcode;
  iCounter.byte.lo = iMaximumNumberOfInstructions;
  iCounter.byte.hi = iMaximumNumberOfInstructions ? 0 : 1;
  m_ControlFlags = 0;
  do
  {
    opcode.word = ReadProgramWord(m_pc.word);   
#ifdef AVRVIRT_DEBUG
    instrFetch();
#endif
    decodeInstruction(opcode);
#ifdef AVRVIRT_DEBUG
      instrExecuted();
      dbgLine(m_dbgLine);
#endif
    
    if (m_ControlFlags & (uint8_t)eAttention)
    {
      if (m_ControlFlags & (uint8_t)ePCFinished)
      {
	m_ControlFlags = m_ControlFlags & ~(uint8_t)ePCFinished;
      }
      else if (m_ControlFlags & (uint8_t)eIllegalOpcode)
      {
	break;
      }
      else
      {
	++m_pc.word;
      }
      
      if (m_ControlFlags & (uint8_t)(eBreakOpcode | eSleepOpcode | eWDROpcode))
      {
	break;
      }
      
      if ((m_ControlFlags & ~(uint8_t)eAttention) == 0)
      {
	m_ControlFlags = 0;
      }
    }
    else
    {
      ++m_pc.word;
    }
    --iCounter.byte.lo;
  }while(iCounter.word != 0);
  if (iMaximumNumberOfInstructions)
  {
    return iMaximumNumberOfInstructions - iCounter.byte.lo;
  }
  else
  {
    return 0;
  }
}
//-----------------------------------------------------------------------------
void VM::decodeInstruction(const avr_word_t opcode)
{
  switch(shr4(opcode.byte.hi))
  {
    case 0b0000u:
      switch(opcode.byte.hi & (uint8_t)0x0Fu)
      {
	case 0b0000u: //NOP
	  if (opcode.byte.lo != 0) //!NOP
	    illegalOpcode(opcode);
#ifdef AVRVIRT_DEBUG
	    else DECODED("%s","NOP")
#endif
	  break;
	case 0b0001u: //MOVW
	  instr_movw(opcode);
	  break;
	case 0b0010u: //MULS
	  instr_muls(opcode);
	  break;
	case 0b0011u: //MULS, MULSU, FMUL, FMULSU
	  instr_fmul(opcode);
	  break;
	default:
	  switch(opcode.byte.hi >> 2)
	  {
	    case 0b01u: //CPC
	      instr_reg_reg_im8(opcode,0x04u); 
	      break;
	    case 0b10u: //SBC
	      instr_reg_reg_im8(opcode,0x08u);
	      break;
	    case 0b11u: //ADD
	      instr_reg_reg_im8(opcode,0x01u);
	      break;
	    default: //never reached
	      break;
	  }
      }
      break;
    case 0b0001u:
	  switch((opcode.byte.hi >> 2) & (uint8_t)0b11u)
	  {
	    case 0b00u: //CPSE
	      instr_cpse(opcode);
	      break;
	    case 0b01u: //CP
	      instr_reg_reg_im8(opcode,0x03u); 
	      break;
	    case 0b10u: //SUB
	      instr_reg_reg_im8(opcode,0x09u);
	      break;
	    case 0b11u: //ADC
	      instr_reg_reg_im8(opcode,0x00u);
	      break;
	    default: //never reached
	      break;
	  }
	  break;
    case 0b0010u:
	  switch((opcode.byte.hi >> 2) & (uint8_t)0b11u)
	  {
	    case 0b00u: //AND
	      instr_reg_reg_im8(opcode,0x02u);
	      break;
	    case 0b01u: //EOR
	      instr_reg_reg_im8(opcode,0x05u); 
	      break;
	    case 0b10u: //OR
	      instr_reg_reg_im8(opcode,0x07u);
	      break;
	    case 0b11u: //MOV
	      instr_reg_reg_im8(opcode,0x06u);
	      break;
	    default: //never reached
	      break;
	  }
	  break;
    case 0b0011u: //CPI
      instr_reg_reg_im8(opcode,0x13u);
      break;
    case 0b0100u: //SBCI
      instr_reg_reg_im8(opcode,0x18u);
      break;
    case 0b0101u: //SUBI
      instr_reg_reg_im8(opcode,0x19u);
      break;
    case 0b0110u: //ORI
      instr_reg_reg_im8(opcode,0x17u);
      break;
    case 0b0111u: //ANDI
      instr_reg_reg_im8(opcode,0x12u);
      break;
    case 0b1000u: //LDD/STD
      instr_ldd_std(opcode);
      break;
    case 0b1001u:
      switch((opcode.byte.hi & (uint8_t)0b1110u) >> 1)
      {
	case 0b000u:
	case 0b001u:
	  switch((opcode.byte.lo & (uint8_t)0x0Fu) | (uint8_t)((opcode.byte.hi & (uint8_t)0b0010u) ? 0x10u : 0u))
	  {
	    case 0b00000u: //LDS/STS (32bit)
	    case 0b10000u: //LDS/STS (32bit)
	      instr_lds_sts(opcode);
	      break;
	    case 0b00001u: //LD/ST
	    case 0b01001u:
	    case 0b00010u:
	    case 0b01010u:
	    case 0b10001u:
	    case 0b11001u:
	    case 0b10010u:
	    case 0b11010u:
	      instr_ld_st(opcode);
	      break;
	    case 0b00100u: //LPM Rd,Z
	      instr_lpm(opcode);
	      break;
	    case 0b00101u: //LPM Rd,Z+
	      instr_lpm(opcode);
	      break;
	    case 0b01100u: //LD/ST
	    case 0b01101u:
	    case 0b01110u:
	    case 0b11100u:
	    case 0b11101u:
	    case 0b11110u:
	      instr_ld_st(opcode);
	      break;
	    case 0b01111u:
	    case 0b11111u:
	      instr_pop_push(opcode);
	      break;
	    default:
	      illegalOpcode(opcode); //Not supported: ELPM, XCH, LAS, LAC, LAT
	      break;
	  }
	  break;
	case 0b010u:
	  if (opcode.byte.lo & (uint8_t)0b1000u)
	  {
	      switch(opcode.byte.lo & (uint8_t)0b111u)
	      {
		case 0b000u:
		  if (opcode.byte.hi & (uint8_t)1u)
		  {
		    instr_misc(opcode);
		  }
		  else
		  {
		    instr_bclr_bset(opcode);
		  }
		  break;
		case 0b001u:
		  if (opcode.byte.lo & (uint8_t)0b10000u) //EIJMP/EICALL
		  {
		    illegalOpcode(opcode);
		  }
		  else
		  {
		    instr_icall_ijmp(opcode); //IJMP/ICALL
		  }
		  break;
		case 0b010u: //DEC
		  instr_reg(opcode);
		  break;
		case 0b011u: //DES
		  illegalOpcode(opcode);
		  break;
		default: //JMP/CALL
		  instr_call_jmp(opcode);
		  break;
	      }
	  }
	  else
	  {
	    instr_reg(opcode); //COM,NEG,SWAP,INC,ASR,LSR,ROR
	  }
	  break;
	case 0b011u: //ADIW, SBIW
	  instr_16im(opcode);
	  break;
	case 0b100u:
	case 0b101u:
	  instr_cbi_sbi(opcode);
	  break;
	case 0b110u:
	case 0b111u:
	  instr_mul(opcode);
	  break;
      }
      break;
    case 0b1010u: //LDD/STD
      instr_ldd_std(opcode);
      break;
    case 0b1011u: //IN/OUT
      instr_in_out(opcode);
      break;
    case 0b1100u:
    case 0b1101u:
      instr_rcall_rjmp(opcode);
      break;
    case 0b1110u:
      instr_ldi(opcode);
      break;
    case 0b1111u:
      if(opcode.byte.hi & (uint8_t)0b1000u)
      {
	instr_bld_bst(opcode);
      }
      else
      {
	instr_br(opcode);
      }
      break;
    default:
      break;
  }
}