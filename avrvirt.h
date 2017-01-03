/**
 * \mainpage
 * \author Attila Laszlo Agas
 * \date 2017-01-02
 * \brief AVR Virt is a software virtualization solution for the AVR family of microcontrollers.
 * 
 *
 * \section intro_sec Introduction
 *
 * AVR Virt is an Arduino library made to emulate the AVR cpu core.
 * The current implementation uses inline assembly and depends on AVR instructions so it can only be compiled to 
 * Atmel AVR based devices. In case of sufficient interest I would make it cross platform.
 * 
 * Features:
 * * Simple to use
 * * Fits into atmega8 program memory
 * * Object oriented, class \ref VM can be inherited and extended.
 * * By subclassing it is possible to run code from any media (SRAM, EEPROM, external memories, network)
 * * Multiple instances of \ref VM can be running.
 * * Guest opcodes can be run even one by one.
 * * It can run itself so nested virtual machines are possible :)
 *
 * \section license_sec License
 * Copyright (c) 2017, Attila Laszlo Agas (http://www.hwelectrons.com)
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

#ifndef AVRVIRT_H
#define AVRVIRT_H

#ifndef __AVR_ARCH__
#error "Sorry, AVR Virt is not implemented for this architecture."
#endif

#include <stdint.h>
#include <avr/pgmspace.h>

//#define AVRVIRT_FASTMEM 1
//#define AVRVIRT_DEBUG 1
#define AVRVIRT_DEBUG_BUFFERSIZE 160

namespace avrvirt
{
  /**
   * \brief Union type to simplify word/low byte/high byte accesses.
   */
  typedef union
  {
    uint16_t word; ///< Access word
    struct
    {
      uint8_t lo; ///< Access low byte
      uint8_t hi; ///< Access high byte
    } byte; ///< Access byte
  } avr_word_t;

  /**
   * \enum avrvirt::EControlFlags
   * \brief Control flags are used both internally and externally.
   */
  enum EControlFlags
  { 
    /// This flag must be set when any other flag is set.
    eAttention = 1,

    /// This flag is used internally. It means the current instruction has set up a new PC
    /// (eg.: CALL, RET) and no automatic increment is required.
    ePCFinished = 2,

    /// VM halted due to an illegal opcode.
    /// (VM::run() returns immediately and PC points to instruction.)
    eIllegalOpcode = 4,

    /// BREAK instruction was executed.
    /// (VM::run() returns immediately and PC points to instruction.)
    eBreakOpcode = 8,

    /// SLEEP instruction was executed.
    /// (VM::run() returns immediately and PC points to instruction.)
    eSleepOpcode = 16,

    /// WDR instruction was executed.
    /// (VM::run() returns immediately and PC points to instruction.)
    eWDROpcode = 32                       
  }; 
  

  /**
   * \brief Implementation of the AVR virtual machine.
   * 
   * You may want to inherit your own class from it and override the following methods:
   * \ref ReadDataByte, \ref WriteDataByte, \ref ReadProgramByte, \ref ReadProgramWord
   * 
   * See http://www.hwelectrons.com/?q=avrvirt for examples.
   */
  class VM
  {
  public:
    /**
     * \brief Constructor
     */
    VM();
    
    /**
     * \brief Destructor
     */
    ~VM() {}
    
    /**
     * \brief Starts the virtual machine.
     * 
     * Call this method in a loop. It returns when one or more of the following conditions is occured:
     * * An illegal opcode is reached.
     * * A BREAK opcode is reached.
     * * A SLEEP opcode is reached.
     * * A WDR opcode is reached.
     * * \p iMaximumNumberOfInstructions is positive and corresponding number of instructions has been processed.
     * 
     * \param iMaximumNumberOfInstructions is the upper limit for the number of instructions to be processed in one run.
     * 0 means there is no upper limit.
     * 
     * \return The number of instructions processed or 0 when iMaximumNumberOfInstructions equals 0.
     */
    uint8_t run(const uint8_t iMaximumNumberOfInstructions = 0);
    
    /**
     * \brief Returns the control flags.
     * 
     * See \ref EControlFlags. It can be used to determine the reason why \ref run method was terminated.  
     */
    const uint8_t getControlFlags() const { return m_ControlFlags; }
    
    /**
     * \brief Sets the SP register
     * 
     * \param iAddress is a byte address in the range 0..65535.
     */ 
    void setSP(const uint16_t iAddress)
    {
      m_sp.word = iAddress;
    }
    
    /**
     * \brief Returns the value of SP register.
     * 
     * \return Byte address in the range 0..65535.
     */
    const uint16_t getSP() const
    {
      return m_sp.word;
    }
    
    /**
     * \brief Sets the PC register
     * 
     * @param iAddress is a word address in the range 0..65535.
     */
    void setPC(const uint16_t iAddress)
    {
      m_pc.word = iAddress;
    }
    
    /**
     * \brief Returns the value of PC register.
     * 
     * \return Word address in the range 0..65535.
     */
    const uint16_t getPC() const
    {
      return m_pc.word;
    }
    
    /**
     * \brief Returns a reference to the given R register.
     * 
     * \param iIndex is a register index in the range 0..31.
     * \return Reference of the given R register.
     */
    uint8_t& getR(const uint8_t iIndex)
    {
      return m_r[iIndex & 31u];
    }
    
    /**
     * \brief Returns a reference to the SREG.
     * 
     * \return Referene to the SREG.
     */
    uint8_t& getSREG()
    {
      return m_sreg;
    }
  protected:
    /**
     * \brief Method used by virtual machine to read data RAM
     * 
     * The default implementation accesses the SRAM on your mcu. You may want to override this method
     * to restrict and/or expand the address space of your virtual machine.
     * 
     * \param iAddress is a byte address in the range 0..65535.
     * \return Data byte
     */
    virtual const uint8_t ReadDataByte(const uint16_t iAddress);
    
    /**
     * \brief Method used by virtual machine to write data RAM
     * 
     * The default implementation accesses the SRAM on your mcu. You may want to override this method
     * to restrict and/or expand the address space of your virtual machine.
     * 
     * \param iAddress is a byte address in the range 0..65535.
     * \param src is byte value to be written.
     */
    virtual void WriteDataByte(const uint16_t iAddress,const uint8_t src);
    
    /**
     * \brief Method used by virtual machine to read program memory.
     * 
     * This method emulates byte access (eg.: LPM instruction).
     * The default implementation accesses the program flash on your mcu. You may want to override this method
     * to restrict and/or expand the address space of your virtual machine.
     * 
     * \param iAddress is a byte address in the range 0..65535.
     * \return Data byte
     */
    virtual const uint8_t ReadProgramByte(const uint16_t iAddress);
    
    /**
     * \brief Method used by virtual machine to read program memory.
     * 
     * This method emulates word access (eg.: program execution).
     * The default implementation accesses the program flash on your mcu. You may want to override this method
     * to restrict and/or expand the address space of your virtual machine.
     * 
     * \param iAddress is a word address in the range 0..65535.
     * \return Data word
     */
    virtual const uint16_t ReadProgramWord(const uint16_t iAddress);
    
    
#ifdef AVRVM_DEBUG
    /**
     * \brief Convenient function in debug mode
     * 
     * In debug mode this method must be implemented. It is called on execution of every opcode with a nice formatted
     * string containing the address, disassembled instruction, affected registers.
     * 
     * \param pDBGLine formatted debug line.
     */
    virtual void dbgLine(const char *pDBGLine) = 0;
#endif

    
  private:
    /**
     * \brief Internal use only
     */
    const uint8_t ReadDataByte_(const uint16_t iAddress);
    
    /**
     * \brief Internal use only
     */   
    void WriteDataByte_(const uint16_t iAddress,const uint8_t src);
    
    /**
     * \brief Internal use only
     */
    const uint8_t ReadIOByte_(const uint8_t iAddress);
    
    /**
     * \brief Internal use only
     */    
    void WriteIOByte_(const uint8_t iAddress,const uint8_t src);

#ifdef AVRVM_DEBUG
    /**
     * \brief Internal use only
     * 
     * In debug mode it prints the current PC.
     */
    void instrFetch();
    
    /**
     * \brief Internal use only
     * 
     * In debug mode it finalizes the debug line string.
     */    
    void instrExecuted();
#endif
    /**
     * \brief Internal use only
     * 
     * Skips next opcode by increasing PC accordingly (opcode length can be 1 or 2 words).
     */
    void skipNextInstruction();
    
    /**
     * \brief Internal use only
     * 
     * Sets control flags accordingly. In debug mode opcode word is printed.
     */
    void illegalOpcode(const avr_word_t opcode);
    
    /**
     * \brief Internal use only
     * 
     * Sets controls flags accordingly. It is called when the current instruction has set up the new PC and no automatic
     * increment is required.
     */
    void setPCFinished();

    /**
     * \brief Internal use only
     * 
     * Decodes register from opcode.
     * 
     * \param opcode is the opcode word.
     * \return Reference to R register.
     */
    uint8_t& decode_Rr(const avr_word_t opcode);

    /**
     * \brief Internal use only
     * 
     * Decodes register from opcode.
     * 
     * \param opcode is the opcode word.
     * \return Reference to R register.
     */
    uint8_t& decode_Rd(const avr_word_t opcode);

    /**
     * \brief Internal use only
     * 
     * Decodes register from opcode.
     * 
     * \param opcode is the opcode word.
     * \return Reference to R register.
     */
    uint8_t& decode_RdUpper(const avr_word_t opcode);
    
    /**
     * \brief Internal use only
     * 
     * Decodes register from opcode.
     * 
     * \param opcode is the opcode word.
     * \return Reference to R register.
     */    
    uint8_t& decode_RrUpper(const avr_word_t opcode);
    
    /**
     * \brief Internal use only
     * 
     * Decodes register from opcode.
     * 
     * \param opcode is the opcode word.
     * \return Reference to one of the 16 bit register pairs.
     */
    uint16_t& decode_RdUpper4Pairs(const avr_word_t opcode);

    /**
     * \brief Internal use only
     * 
     * Decodes 6 bit literal from opcode.
     * 
     * \param opcode is the opcode word.
     * \return 6 bit literal.
     */
    uint8_t decode_K6Bit(const avr_word_t opcode);
    
    /**
     * \brief Internal use only
     * 
     * Decodes 8 bit literal from opcode.
     * 
     * \param opcode is the opcode word.
     * \return 8 bit literal.
     */
    uint8_t decode_K8Bit(const avr_word_t opcode);
    
    
    /**
     * \brief Internal use only
     * 
     * Decodes register from opcode.
     * 
     * \param opcode is the opcode word.
     * \return Reference to the X,Y or Z register.
     */
    avr_word_t& decode_IndexReg(const avr_word_t opcode);
    
    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * bclr: 10010100 1sss1000
     * * bset: 10010100 0sss1000
     * * clc:  10010100 10001000
     * * clh:  10010100 11011000
     * * cli:  10010100 11111000
     * * cln:  10010100 10101000
     * * cls:  10010100 11001000
     * * clt:  10010100 11101000
     * * clv:  10010100 10111000
     * * clz:  10010100 10011000
     * * sec:  10010100 00001000
     * * seh:  10010100 01011000
     * * sei:  10010100 01111000
     * * sen:  10010100 00101000
     * * ses:  10010100 01001000
     * * set:  10010100 01101000
     * * sev:  10010100 00111000
     * * sez:  10010100 00011000
     * 
     * \param opcode is the opcode word.
     */   
    void instr_bclr_bset(const avr_word_t opcode);
    

    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * bld  1111100d dddd0bbb
     * * bst  1111101d dddd0bbb
     * * sbrc 1111110r rrrr0bbb
     * * sbrs 1111111r rrrr0bbb
     * 
     * \param opcode is the opcode word.
     */   
    void instr_bld_bst(const avr_word_t opcode);
    
    
    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * brbc: 111101kk kkkkksss
     * * brbs: 111100kk kkkkksss
     * * brcs: 111100kk kkkkk000
     * * breq: 111100kk kkkkk001
     * * brge: 111101kk kkkkk100
     * * brhc: 111101kk kkkkk101
     * * brhs: 111100kk kkkkk101
     * * brid: 111101kk kkkkk111
     * * brie: 111100kk kkkkk111
     * * brlo: 111100kk kkkkk000
     * * brlt: 111100kk kkkkk100
     * * brmi: 111100kk kkkkk010
     * * brne: 111101kk kkkkk001
     * * brpl: 111101kk kkkkk010
     * * brsh: 111101kk kkkkk000
     * * brtc: 111101kk kkkkk110
     * * brts: 111100kk kkkkk110
     * * brvc: 111101kk kkkkk011
     * * brvs: 111100kk kkkkk011   
     * 
     * \param opcode is the opcode word.
     */   
    void instr_br(const avr_word_t opcode);
    
    
    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * call: 1001010k kkkk111k kkkkkkkk kkkkkkk
     * * jmp:  1001010k kkkk110k kkkkkkkk kkkkkkk
     * 
     * \param opcode is the opcode word.
     */   
    void instr_call_jmp(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * cbi: 10011000 AAAAAbbb
     * * sbi: 10011010 AAAAAbbb
     * * sbic 10011001 AAAAAbbb
     * * sbis 10011011 AAAAAbbb
     * 
     * \param opcode is the opcode word.
     */   
    void instr_cbi_sbi(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * subcode 0x00:	adc: 000111rd ddddrrrr
     * * subcode 0x01:	add: 000011rd ddddrrrr
     * * subcode 0x02:	and: 001000rd ddddrrrr
     * * subcode 0x03:	cp:  000101rd ddddrrrr
     * * subcode 0x04:	cpc: 000001rd ddddrrrr
     * * subcode 0x05:	eor: 001001rd ddddrrrr
     * * subcode 0x06:	mov: 001011rd ddddrrrr
     * * subcode 0x07:	or:  001010rd ddddrrrr
     * * subcode 0x08:	sbc: 000010rd ddddrrrr
     * * subcode 0x09:	sub: 000110rd ddddrrrr
     * * subcode 0x12:	andi: 0111KKKK ddddKKKK
     * * subcode 0x12:	cbr:  andi ~K
     * * subcode 0x13:	cpi:  0011KKKK ddddKKKK
     * * subcode 0x17:	ori:  0110KKKK ddddKKKK
     * * subcode 0x18:	sbci: 0100KKKK ddddKKKK
     * * subcode 0x19:	subi: 0101KKKK ddddKKKK
     * 
     * \param opcode is the opcode word.
     * \param subcode
     */   
    void instr_reg_reg_im8(const avr_word_t opcode, const uint8_t subcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * cpse: 000100rd ddddrrrr   
     * 
     * \param opcode is the opcode word.
     */   
    void instr_cpse(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * adiw: 10010110 KKddKKKK
     * * sbiw: 10010111 KKddKKKK
     * 
     * \param opcode is the opcode word.
     */   
    void instr_16im(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * asr Rd:  1001010d dddd0101
     * * lsr Rd:  1001010d dddd0110
     * * com Rd:  1001010d dddd0000
     * * dec Rd:  1001010d dddd1010
     * * inc Rd:  1001010d dddd0011
     * * neg Rd:  1001010d dddd0001
     * * ror Rd:  1001010d dddd0111
     * * swap Rd: 1001010d dddd0010
     * 
     * \param opcode is the opcode word.
     */   
    void instr_reg(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * mul: 100111rd ddddrrrr
     * 
     * \param opcode is the opcode word.
     */   
    void instr_mul(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * muls: 00000010 ddddrrrr
     * 
     * \param opcode is the opcode word.
     */   
    void instr_muls(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * mulsu:  00000011 0ddd0rrr
     * * fmul:   00000011 0ddd1rrr
     * * fmuls:  00000011 1ddd0rrr
     * * fmulsu: 00000011 1ddd1rrr
     * 
     * \param opcode is the opcode word.
     */   
    void instr_fmul(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * icall: 10010101 00001001
     * * ijmp:  10010100 00001001
     * 
     * \param opcode is the opcode word.
     */   
    void instr_icall_ijmp(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * in:	 10110AAd ddddAAAA
     * * out:   10111AAr rrrrAAAA
     * 
     * \param opcode is the opcode word.
     */   
    void instr_in_out(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * pop:  1001000d dddd1111
     * * push: 1001001d dddd1111
     * 
     * \param opcode is the opcode word.
     */   
    void instr_pop_push(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * ld Rd,X:	1001000d dddd1100
     * * ld Rd,X+:	1001000d dddd1101
     * * ld Rd,Y+:	1001000d dddd1001
     * * ld Rd,Z+:	1001000d dddd0001
     * * ld Rd,-X:	1001000d dddd1110
     * * ld Rd,-Y:	1001000d dddd1010
     * * ld Rd,-Z:	1001000d dddd0010
     * * st X,Rr:	1001001r rrrr1100
     * * st X+,Rr:	1001001r rrrr1101
     * * st Y+,Rr:	1001001r rrrr1001
     * * st Z+,Rr:	1001001r rrrr0001
     * * st -X,Rr:	1001001r rrrr1110
     * * st -Y,Rr:	1001001r rrrr1010
     * * st -Z,Rr:	1001001r rrrr0010
     * 
     * \param opcode is the opcode word.
     */   
    void instr_ld_st(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * ldd Rd,Y+q:	10q0qq0d dddd1qqq
     * * ldd Rd,Z+q:	10q0qq0d dddd0qqq
     * * std Y+q,Rr:	10q0qq1r rrrr1qqq
     * * std Z+q,Rr:	10q0qq1r rrrr0qqq
     * 
     * \param opcode is the opcode word.
     */   
    void instr_ldd_std(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * ldi Rd,k:	1110kkkk ddddkkkk
     * 
     * \param opcode is the opcode word.
     */   
    void instr_ldi(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * lds Rd,k:  1001000d dddd0000 kkkkkkkk kkkkkkkk
     * * sts k,Rd:  1001001d dddd0000 kkkkkkkk kkkkkkkk
     * 
     * \param opcode is the opcode word.
     */   
    void instr_lds_sts(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * lds Rd,k:  10100kkk ddddkkkk
     * * sts k,Rd:  10101kkk ddddkkkk
     * 
     * \param opcode is the opcode word.
     */   
    void instr_lds7_sts7(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * lpm Rd,Z:  1001000d dddd0100
     * * lpm Rd,Z+: 1001000d dddd0101
     * 
     * \param opcode is the opcode word.
     */   
    void instr_lpm(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * movw:  00000001 ddddrrrr
     * 
     * \param opcode is the opcode word.
     */   
    void instr_movw(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * rcall k:  1101kkkk kkkkkkkk
     * * rjmp k:   1100kkkk kkkkkkkk
     * 
     * \param opcode is the opcode word.
     */   
    void instr_rcall_rjmp(const avr_word_t opcode);


    /**
     * \brief Internal use only
     * 
     * Handled opcodes:
     * * ret:   	10010101 00001000
     * * reti:  	10010101 00011000
     * * sleep: 	10010101 10001000
     * * wdr:   	10010101 10101000
     * * lpm R0,Z:   	10010101 11001000
     * * break: 	10010101 10011000
     * 
     * \param opcode is the opcode word.
     */   
    void instr_misc(const avr_word_t opcode);

    /**
     * \brief Decodes \p opcode and calls its implementation.
     * 
     * A hand coded switch-case tree is used. In theory compiler should optimize it to jump tables.
     * Look up tables could be used instead but I assume the current implementation consumes less program memory.
     * 
     * \param opcode is an AVR opcode.
     */
    void decodeInstruction(const avr_word_t opcode);
       
    uint8_t m_sreg; ///< SREG (status register)
    uint8_t m_r[32]; ///< R0-R31
    avr_word_t m_pc; ///< PC register
    avr_word_t m_sp; ///< SP register
    avr_word_t& m_X; ///< Reference to the X register
    avr_word_t& m_Y; ///< Reference to the Y register
    avr_word_t& m_Z; ///< Reference to the Z register
    uint8_t m_ControlFlags; ///< See \ref EControlFlags
#ifdef AVRVM_DEBUG
    char m_dbgLine[AVRVIRT_DEBUG_BUFFERSIZE]; ///< Only in debug mode. Buffer for debug text.
#endif
    
   /**
    * \brief Copy constructor is disabled.
    */
   VM(const VM&);
   
   /**
    * \brief Assignment operator is disabled.
    */
   VM& operator=(const VM&);
  };

  /**
   * \brief Shifts right 4 bits.
   * 
   * (Compiler may generate more instructions at least my version did.)
   * 
   * \param b The input byte.
   * \return The input byte shifted right by 4 bits.
   */ 
  inline uint8_t shr4(uint8_t b)
  {
    __asm__ __volatile__ 
    ("swap %0\n\t"
    "andi %0,0x0F\n\t" : "+d" (b) : );
    return b;
  }
  
  /**
   * \brief Shifts left 4 bits.
   * 
   * (Compiler may generate more instructions at least my version did.)
   * 
   * \param b The input byte.
   * \return The input byte shifted left by 4 bits.
   */  
  inline uint8_t shl4(uint8_t b)
  {
    __asm__ __volatile__ 
    ("swap %0\n\t"
    "andi %0,0xF0\n\t" : "+d" (b) : );
    return b;
  }
  
  /**
   * \brief Sets the given bit to 1.
   * 
   * \param iIndex The index of the bit to be set. (0..7)
   * \return A byte where bit at iIndex position is set to 1.
   */
  inline const uint8_t setBit(const uint8_t iIndex)
  {
    static const uint8_t iMask[8] PROGMEM ={1,2,4,8,16,32,64,128};
    return pgm_read_byte(&iMask[iIndex]);
  }

  
} //namespace avrvirt

#endif //AVRVIRT_H