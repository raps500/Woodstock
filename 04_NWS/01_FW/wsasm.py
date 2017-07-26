#!/usr/bin/python

import sys

TYPE0_OPS = [
                    [ 0b0000000000, "nop" ],
                    [ 0b0001000000, "crc100" ],
                    [ 0b0010000000, "nop" ],
                    [ 0b0011000000, "crc300" ],
                    [ 0b0100000000, "crc400" ],
                    [ 0b0101000000, "crc500" ],
                    [ 0b0110000000, "nop" ],
                    [ 0b0111000000, "nop" ],
                    [ 0b1000000000, "crc1000" ],
                    [ 0b1001000000, "crc1100" ],
                    [ 0b1010000000, "crc1200" ],
                    [ 0b1011000000, "crc1300" ],
                    [ 0b1100000000, "crc1400" ],
                    [ 0b1101000000, "crc1500" ],
                    [ 0b1110000000, "nop" ],
                    [ 0b1111000000, "crc1700" ],
                    
                    [ 0b0000000100, "1 -> s 0" ],  [ 0b0001000100, "1 -> s 1" ],  [ 0b0010000100, "1 -> s 2" ],  [ 0b0011000100, "1 -> s 3" ],
                    [ 0b0100000100, "1 -> s 4" ],  [ 0b0101000100, "1 -> s 5" ],  [ 0b0110000100, "1 -> s 6" ],  [ 0b0111000100, "1 -> s 7" ],
                    [ 0b1000000100, "1 -> s 8" ],  [ 0b1001000100, "1 -> s 9" ],  [ 0b1010000100, "1 -> s 10" ], [ 0b1011000100, "1 -> s 11" ],
                    [ 0b1100000100, "1 -> s 12" ], [ 0b1101000100, "1 -> s 13" ], [ 0b1110000100, "1 -> s 14" ], [ 0b1111000100, "1 -> s 15" ],
                    
                    [ 0b0000001000, "clrregs" ],
                    [ 0b0001001000, "clrstatus" ],
                    [ 0b0010001000, "disptoggle" ],
                    [ 0b0011001000, "dispoff" ],
                    [ 0b0100001000, "c ex m1" ],
                    [ 0b0101001000, "m1 -> c" ],
                    [ 0b0110001000, "c ex m2" ],
                    [ 0b0111001000, "m2 -> c" ],
                    [ 0b1000001000, "popa" ],
                    [ 0b1001001000, "rotstack" ],
                    [ 0b1010001000, "y -> a" ],
                    [ 0b1011001000, "pushc" ],
                    [ 0b1100001000, "decimal" ],
                    [ 0b1110001000, "f -> a[0]" ],
                    [ 0b1111001000, "f ex a[0]" ],
                    
                    [ 0b0000001100, "0 -> s 0" ],  [ 0b0001001100, "0 -> s 1" ],  [ 0b0010001100, "0 -> s 2" ],  [ 0b0011001100, "0 -> s 3" ],
                    [ 0b0100001100, "0 -> s 4" ],  [ 0b0101001100, "0 -> s 5" ],  [ 0b0110001100, "0 -> s 6" ],  [ 0b0111001100, "0 -> s 7" ],
                    [ 0b1000001100, "0 -> s 8" ],  [ 0b1001001100, "0 -> s 9" ],  [ 0b1010001100, "0 -> s 10" ], [ 0b1011001100, "0 -> s 11" ],
                    [ 0b1100001100, "0 -> s 12" ], [ 0b1101001100, "0 -> s 13" ], [ 0b1110001100, "0 -> s 14" ], [ 0b1111001100, "0 -> s 15" ],
                    
                    [ 0b0000010000, "keys -> rom" ],
                    [ 0b0001010000, "keys -> a[2:1]" ],
                    [ 0b0010010000, "a[2:1] -> rom" ],
                    [ 0b0011010000, "disp reset" ],
                    [ 0b0100010000, "hex" ],
                    [ 0b0101010000, "rotate a left" ],
                    [ 0b0110010000, "p - 1 -> p" ],
                    [ 0b0111010000, "p + 1 -> p" ],
                    [ 0b1000010000, "return" ],
                    
                    [ 0b0000010100, "if 1 = s 0" ],  [ 0b0001010100, "if 1 = s 1" ],  [ 0b0010010100, "if 1 = s 2" ],  [ 0b0011010100, "if 1 = s 3" ],
                    [ 0b0100010100, "if 1 = s 4" ],  [ 0b0101010100, "if 1 = s 5" ],  [ 0b0110010100, "if 1 = s 6" ],  [ 0b0111010100, "if 1 = s 7" ],
                    [ 0b1000010100, "if 1 = s 8" ],  [ 0b1001010100, "if 1 = s 9" ],  [ 0b1010010100, "if 1 = s 10" ], [ 0b1011010100, "if 1 = s 11" ],
                    [ 0b1100010100, "if 1 = s 12" ], [ 0b1101010100, "if 1 = s 13" ], [ 0b1110010100, "if 1 = s 14" ], [ 0b1111010100, "if 1 = s 15" ],
                    
                    [ 0b0000011000, "load constant 0" ],  [ 0b0001011000, "load constant 1" ],  [ 0b0010011000, "load constant 2" ],  [ 0b0011011000, "load constant 3" ],
                    [ 0b0100011000, "load constant 4" ],  [ 0b0101011000, "load constant 5" ],  [ 0b0110011000, "load constant 6" ],  [ 0b0111011000, "load constant 7" ],
                    [ 0b1000011000, "load constant 8" ],  [ 0b1001011000, "load constant 9" ],  [ 0b1010011000, "load constant 10" ], [ 0b1011011000, "load constant 11" ],
                    [ 0b1100011000, "load constant 12" ], [ 0b1101011000, "load constant 13" ], [ 0b1110011000, "load constant 14" ], [ 0b1111011000, "load constant 15" ],
                    
                    [ 0b0000011100, "if 0 = s 0" ],  [ 0b0001011100, "if 0 = s 1" ],  [ 0b0010011100, "if 0 = s 2" ],  [ 0b0011011100, "if 0 = s 3" ],
                    [ 0b0100011100, "if 0 = s 4" ],  [ 0b0101011100, "if 0 = s 5" ],  [ 0b0110011100, "if 0 = s 6" ],  [ 0b0111011100, "if 0 = s 7" ],
                    [ 0b1000011100, "if 0 = s 8" ],  [ 0b1001011100, "if 0 = s 9" ],  [ 0b1010011100, "if 0 = s 10" ], [ 0b1011011100, "if 0 = s 11" ],
                    [ 0b1100011100, "if 0 = s 12" ], [ 0b1101011100, "if 0 = s 13" ], [ 0b1110011100, "if 0 = s 14" ], [ 0b1111011100, "if 0 = s 15" ],
                    
                    [ 0b0000100000, "select rom 0" ],  [ 0b0001100000, "select rom 1" ],  [ 0b0010100000, "select rom 2" ],  [ 0b0011100000, "select rom 3" ],
                    [ 0b0100100000, "select rom 4" ],  [ 0b0101100000, "select rom 5" ],  [ 0b0110100000, "select rom 6" ],  [ 0b0111100000, "select rom 7" ],
                    [ 0b1000100000, "select rom 8" ],  [ 0b1001100000, "select rom 9" ],  [ 0b1010100000, "select rom 10" ], [ 0b1011100000, "select rom 11" ],
                    [ 0b1100100000, "select rom 12" ], [ 0b1101100000, "select rom 13" ], [ 0b1110100000, "select rom 14" ], [ 0b1111100000, "select rom 15" ],
                    
                    [ 0b0000100100, "if p = 4" ], [ 0b0001100100, "if p = 8" ], [ 0b0010100100, "if p = 12" ],[ 0b0011100100, "if p = 2" ],
                    [ 0b0100100100, "if p = 9" ], [ 0b0101100100, "if p = 1" ], [ 0b0110100100, "if p = 6" ], [ 0b0111100100, "if p = 3" ],
                    [ 0b1000100100, "if p = 1b"], [ 0b1001100100, "if p = 3" ], [ 0b1010100100, "if p = 13" ],[ 0b1011100100, "if p = 6" ],
                    [ 0b1100100100, "if p = 0" ], [ 0b1101100100, "if p = 9" ], [ 0b1110100100, "if p = 5" ], [ 0b1111100100, "if p = 14" ],
                    
                    [ 0b0000101100, "if p # 4" ],  [ 0b0001101100, "if p # 8" ], [ 0b0010101100, "if p # 12" ], [ 0b0011101100, "if p # 2" ],
                    [ 0b0100101100, "if p # 9" ],  [ 0b0101101100, "if p # 1" ], [ 0b0110101100, "if p # 6"  ], [ 0b0111101100, "if p # 3" ],
                    [ 0b1000101100, "if p # 1b" ], [ 0b1001101100, "if p # 3" ], [ 0b1010101100, "if p # 13" ], [ 0b1011101100, "if p # 6" ],
                    [ 0b1100101100, "if p # 0" ],  [ 0b1101101100,  "if p # 9" ],[ 0b1110101100, "if p # 5"  ], [ 0b1111101100, "if p # 14" ],
                    
                    [ 0b0000101000, "c -> data register 0" ],  [ 0b0001101000, "c -> data register 1" ],  [ 0b0010101000, "c -> data register 2" ],  [ 0b0011101000, "c -> data register 3" ],
                    [ 0b0100101000, "c -> data register 4" ],  [ 0b0101101000, "c -> data register 5" ],  [ 0b0110101000, "c -> data register 6" ],  [ 0b0111101000, "c -> data register 7" ],
                    [ 0b1000101000, "c -> data register 8" ],  [ 0b1001101000, "c -> data register 9" ],  [ 0b1010101000, "c -> data register 10" ], [ 0b1011101000, "c -> data register 11" ],
                    [ 0b1100101000, "c -> data register 12" ], [ 0b1101101000, "c -> data register 13" ], [ 0b1110101000, "c -> data register 14" ], [ 0b1111101000, "c -> data register 15" ],
                    
                    [ 0b0000110000, "crc060" ],
                    [ 0b0001110000, "crc160" ],
                    [ 0b0010110000, "crc260" ],
                    [ 0b0011110000, "crc360" ],
                    [ 0b0100110000, "crc460" ],
                    [ 0b0101110000, "crc560" ],
                    [ 0b0110110000, "crc660" ],
                    [ 0b0111110000, "crc760" ],
                    
                    [ 0b1000110000, "bankswitch" ],
                    [ 0b1001110000, "c -> dataaddress" ],
                    [ 0b1010110000, "clrdregs" ],
                    [ 0b1011110000, "c-> data" ],
                    [ 0b1111110000, "woodstock" ],
                    
                    [ 0b0000110100, "delayed rom 0" ], [ 0b0001110100, "delayed rom 1" ], [ 0b0010110100, "delayed rom 2" ], [ 0b0011110100, "delayed rom 3" ],
                    [ 0b0100110100, "delayed rom 4" ], [ 0b0101110100, "delayed rom 5" ], [ 0b0110110100, "delayed rom 6" ], [ 0b0111110100, "delayed rom 7" ],
                    [ 0b1000110100, "delayed rom 8" ], [ 0b1001110100, "delayed rom 9" ], [ 0b1010110100, "delayed rom 10" ], [ 0b1011110100, "delayed rom 11" ],
                    [ 0b1100110100, "delayed rom 12" ], [ 0b1101110100, "delayed rom 13" ], [ 0b1110110100, "delayed rom 14" ], [ 0b1111110100, "delayed rom 15" ],
                    # next opcode only for HP-25
                    #[ 0b00001110: begin decftfr = 1'b1 ], decregdst = `DSTC ], decregop1 = { 1'b0, DATAADDR } ], end // reg[DA] -> C
                    
                    [ 0b0000111000, "data register 0 -> c" ],  [ 0b0001111000, "data register 1 -> c" ],  [ 0b0010111000, "data register 2 -> c" ],  [ 0b0011111000, "data register 3 -> c" ], 
                    [ 0b0100111000, "data register 4 -> c" ],  [ 0b0101111000, "data register 5 -> c" ],  [ 0b0110111000, "data register 6 -> c" ],  [ 0b0111111000, "data register 7 -> c" ], 
                    [ 0b1000111000, "data register 8 -> c" ],  [ 0b1001111000, "data register 9 -> c" ],  [ 0b1010111000, "data register 10 -> c" ], [ 0b1011111000, "data register 11 -> c" ],
                    [ 0b1100111000, "data register 12 -> c" ], [ 0b1101111000, "data register 13 -> c" ], [ 0b1110111000, "data register 14 -> c" ], [ 0b1111111000, "data register 15 -> c" ],
                    # Load Constant to P, carry cleared from regp module
                    [ 0b0000111100, "14 -> p" ], [ 0b0001111100, "4 -> p" ], [ 0b0010111100, "7 -> p" ],  [ 0b0011111100, "8 -> p" ],
                    [ 0b0100111100, "11 -> p" ], [ 0b0101111100, "2 -> p" ], [ 0b0110111100, "10 -> p" ], [ 0b0111111100, "12 -> p" ],
                    [ 0b1000111100, "1 -> p" ],  [ 0b1001111100, "3 -> p" ], [ 0b10101111,  "13 -> p" ],  [ 0b1011111100, "6 -> p" ],
                    [ 0b1100111100, "0 -> p" ],  [ 0b1101111100, "9 -> p" ], [ 0b1110111100, "5 -> p" ],  [ 0b1111111100, "14b -> p" ],
                    # allow both forms
                    [ 0b0000111100, "p <- 14" ], [ 0b0001111100, "p <- 4" ], [ 0b0010111100, "p <- 7" ],  [ 0b0011111100, "p <- 8" ],
                    [ 0b0100111100, "p <- 11" ], [ 0b0101111100, "p <- 2" ], [ 0b0110111100, "p <- 10" ], [ 0b0111111100, "p <- 12" ],
                    [ 0b1000111100, "p <- 1" ],  [ 0b1001111100, "p <- 3" ], [ 0b1010111100, "p <- 13" ], [ 0b1011111100, "p <- 6" ],
                    [ 0b1100111100, "p <- 0" ],  [ 0b1101111100, "p <- 9" ], [ 0b1110111100, "p <- 5" ],  [ 0b1111111100, "p <- 14b" ]
]
ALU_OPS = [
 [ 0b00000, '0 -> a',        ], 
 [ 0b00001, '0 -> b',        ], 
 [ 0b00010, 'a ex b',        ], 
 [ 0b00011, 'a -> b',        ], 
 [ 0b00100, 'a ex c',        ], 
 [ 0b00101, 'c -> a',        ], 
 [ 0b00110, 'b -> c',        ], 
 [ 0b00111, 'b ex c',        ], 
 [ 0b01000, '0 -> c',        ], 
 [ 0b01001, 'a + b -> a',    ], 
 [ 0b01010, 'a + c -> a',    ], 
 [ 0b01011, 'c + c -> c',    ], 
 [ 0b01100, 'a + c -> c',    ], 
 [ 0b01101, 'a + 1 -> a',    ], 
 [ 0b01110, 'shift left a',  ], 
 [ 0b01111, 'c + 1 -> c',    ], 
 [ 0b10000, 'a - b -> a',    ], 
 [ 0b10001, 'a - c -> c',    ], 
 [ 0b10010, 'a - 1 -> a',    ], 
 [ 0b10011, 'c - 1 -> c',    ], 
 [ 0b10100, '0 - c -> c',    ], 
 [ 0b10101, '0 - c - 1 -> c',], 
 [ 0b10110, 'if 0 = b',      ], 
 [ 0b10111, 'if 0 = c',      ], 
 [ 0b11000, 'if a >= c',     ], 
 [ 0b11001, 'if a >= b',     ], 
 [ 0b11010, 'if 0 # a',      ], 
 [ 0b11011, 'if 0 # c',      ], 
 [ 0b11100, 'a - c -> a',    ], 
 [ 0b11101, 'shift right a', ], 
 [ 0b11110, 'shift right b', ], 
 [ 0b11111, 'shift right c'  ] ]
 
SYM_TABLE = dict()

def addSymbol(symName, Addr):
    if symName not in SYM_TABLE:
        SYM_TABLE[symName] = Addr
        return True
    return False

def getOpcode(opName, lineNum, noSymbolLookup):
    oo = opName.split()
    
    if len(opName) < 1:
        return []
        
    if oo[0] == "gonc":
        if noSymbolLookup:
            return [ 3 ] # placeholder
        if oo[1] not in SYM_TABLE:
            print 'Symbol >{0:s}< not found at line {1:d}'.format(oo[1], lineNum)
            return None
        return [ (SYM_TABLE[oo[1]] << 2) | 3 ]
    elif oo[0] == "gsb":
        if noSymbolLookup:
            return [ 1 ] # placeholder
        if oo[1] not in SYM_TABLE:
            print 'Symbol >{0:s}< not found at line {1:d}'.format(oo[1], lineNum)
            return None
        return [ (SYM_TABLE[oo[1]] << 2) | 1 ]
    elif oo[0] == "goyes":
        if noSymbolLookup:
            return [ 0 ] # placeholder
        if oo[1] not in SYM_TABLE:
            print 'Symbol >{0:s}< not found at line {1:d}'.format(oo[1], lineNum)
            return None
        return [ SYM_TABLE[oo[1]] ]
    
    for op in TYPE0_OPS:
        if opName == op[1]:
            return [ op[0] ]
    
    if '[' not in opName or ']' not in opName:
        print 'Unknown type 0, 1 or 3 opcode >{0:s}< at line {1:d}'.format(opName, lineNum)
        return None
     
    oo = opName.split('[')
    oo[0] = oo[0].strip() # remove  extra space
    oo[1] = '[' + oo[1]
    # all alu opcodes have a size field
    for op in ALU_OPS:
        if oo[0] == op[1]:
            #print oo, op
            if '[p]' in oo[1]:
                return [ (op[0] << 5) | ( 0 << 2) | 2 ]
            elif '[wp]' in oo[1]:
                return [ (op[0] << 5) | ( 1 << 2) | 2 ]
            elif '[xs]' in oo[1]:
                return [ (op[0] << 5) | ( 2 << 2) | 2 ]
            elif '[x]' in oo[1]:
                return [ (op[0] << 5) | ( 3 << 2) | 2 ]
            elif '[s]' in oo[1]:
                return [ (op[0] << 5) | ( 4 << 2) | 2 ]
            elif '[m]' in oo[1]:
                return [ (op[0] << 5) | ( 5 << 2) | 2 ]
            elif '[w]' in oo[1]:
                return [ (op[0] << 5) | ( 6 << 2) | 2 ]
            elif '[ms]' in oo[1]:
                return [ (op[0] << 5) | ( 7 << 2) | 2 ]
    
    print 'Unknown type 2 opcode >{0:s}< at line {1:d}'.format(opName, lineNum)
    return None
    
def asmLine(line, lineNum, PC, createSymbol):

    line = line.split(';') # strip comments
    line = line[0].lstrip().rstrip(' \r\n') # strip spaces
    
    if ':' in line:
        sline = line.split(':')
        
        if createSymbol:
            if not addSymbol(sline[0], PC):
                print 'Duplicated symbol >{0:s}< at line {1:d}'.format(sline[0], lineNum)
                return None
        
        line = sline[1]
    op = getOpcode(line, lineNum, createSymbol)
    return op
    
    
def asmFile(fileName):

    lst = open(fileName + '.lst', 'wt')
    bin = open(fileName + '.bin', 'wt')
    
    fi = open(fileName, 'rt')
    
    # first pass
    
    PC = 0
    lineNum = 1
    lines = fi.readlines()
    for j in range(len(lines)):
        lines[j] = lines[j].rstrip().rstrip('\r').rstrip('\n').rstrip('\r')
        
    for line in lines:
        op = asmLine(line, PC, lineNum, True)
        lineNum += 1
        if op == None:
            print 'Pass one: Assembling cancelled due to error'
            quit()
        PC += len(op)
    print 'Woodstock assembler pass one done {0:d} lines'.format(lineNum)
    lineNum = 1  
    PC = 0

    for line in lines:
        op = asmLine(line, PC, lineNum, False)
        if op == None:
            print 'Pass two: Assembling cancelled due to error'
            quit()
        if len(op) != 0:
            lst.write('{0:4d} {1:03x} {2:010b} {3:s}\n'.format(lineNum, PC, op[0], line) )
            bin.write('{0:010b}\n'.format(op[0]))
            PC += len(op)
        lineNum += 1
        
    print 'Woodstock assembler pass one done {0:d} words'.format(PC)        
print 'Woodstock assembler'
asmFile(sys.argv[1])

