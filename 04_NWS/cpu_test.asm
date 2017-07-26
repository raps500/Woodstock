1100001000     decimal
0000011011     if n/c go to 0006
0000000000     nop
0000001011     if n/c go to 0002 used for error
0000000000     nop
0000010011     if n/c go to 0004
0000011010     0 -> a[w]
1101011010     if 0 # a     test clear register a
0000000010     goyes 002
0100011010     0 -> c[w]
1101111010     if 0 # c     test clear register c
0000000010     goyes 002
1010111100     p <- 13
1001011000     load constant 9
1101111010     if 0 # c     test load register c
0000010000     goyes 16d
0010101100     if p # 12    test p compare for not equal
0000000010     goyes 2d
0010100100     if p = 12    test p compare for equal
0000010100     goyes 20d
1001011000     load constant 9
1001011000     load constant 9
1001011000     load constant 9
1001011000     load constant 9
1001011000     load constant 9
1001011000     load constant 9
1001011000     load constant 9
1001011000     load constant 9
1001011000     load constant 9
1001011000     load constant 9
1001011000     load constant 9
1001011000     load constant 9
1001011000     load constant 9
0010111010     c -> a[w]
0001111010     a -> b[w]
0101010000     rotate a left
1101011010     if a[w] # 0
0000100111     goyes 39d
0000001011     if n/c go to 0002 rotate or cmp a # 0 fail
0110101110     a + 1 -> a[x]
1101001110     if a[x] # 0
0000000010     go to 2           a[x] + 1 error
0110110110     a + 1 -> a[m]
1101010110     if a[m] # 0
0000000010     go to 2           a[m] + 1 error
0110110010     a + 1 -> a[s]
1101010010     if a[s] # 0
0000000010     go to 2           a[s] + 1 error
0111111010     c + 1 -> c[w]
1101111010     if c[w] # 0
0000000010     go to 2           c[w] + 1 error
0100011010     0 -> c[w]
1010111100     p <- 13
1001011000     load constant 9   test registers
1111101000     c -> data register 15
1111111010     shift right c[w]
1110101000     c -> data register 14
1111111010     shift right c[w]
1101101000     c -> data register 13
1111111010     shift right c[w]
1100101000     c -> data register 12
1111111010     shift right c[w]
1011101000     c -> data register 11
1111111010     shift right c[w]
1010101000     c -> data register 10
1111111010     shift right c[w]
1001101000     c -> data register 9
1111111010     shift right c[w]
1000101000     c -> data register 8
1111111010     shift right c[w]
0111101000     c -> data register 7
1111111010     shift right c[w]
0110101000     c -> data register 6
1111111010     shift right c[w]
0101101000     c -> data register 5
1111111010     shift right c[w]
0100101000     c -> data register 4
1111111010     shift right c[w]
0011101000     c -> data register 3
1111111010     shift right c[w]
0010101000     c -> data register 2
0000011010     0 -> a[w]
0100011010     0 -> c[w]
0010111000     data register -> c 2
0101011010     a + c -> a[w]
0011111000     data register -> c 3
0101011010     a + c -> a[w]
0100111000     data register -> c 4
0101011010     a + c -> a[w]
0101111000     data register -> c 5
0101011010     a + c -> a[w]
0110111000     data register -> c 6
0101011010     a + c -> a[w]
0111111000     data register -> c 7
0101011010     a + c -> a[w]
1000111000     data register -> c 8
0101011010     a + c -> a[w]
1001111000     data register -> c 9
0101011010     a + c -> a[w]
1010111000     data register -> c 10
0101011010     a + c -> a[w]
1011111000     data register -> c 11
0101011010     a + c -> a[w]
1100111000     data register -> c 12
0101011010     a + c -> a[w]
1101111000     data register -> c 13
0101011010     a + c -> a[w]
1110111000     data register -> c 14
0101011010     a + c -> a[w]
1111111000     data register -> c 15
0101011010     a + c -> a[w]
1000011010     a - b -> a[w] use summed result to clear A
1101011010     if 0 # a
0000000010     goyes 002
0100111010     a + b -> a[w] test decimal add
0110111010     a + 1 -> a[w] clear a
1101011010     if 0 # a
0000000010     goyes 002
0100011010     0 -> c[w]
1001111010     c - 1 -> c    test decimal sub
0010011010     a ex c
1000011010     a - b -> a[w] use summed result to clear A
1101011010     if 0 # a
0000000010     goyes 002
0000011010     0 -> a[w]
0100011010     0 -> c[w]
1001111010     c - 1 -> c    test decimal sub
1000111010     a - c -> c    test reversed sub
0010011010     a ex c
1001011010     a - 1 -> a    test decimal sub on a
1101011010     if 0 # a
0000000010     goyes 002
0100001110     0 -> c[x]     ** test pointer and cmp gtle
1000111100     p <- 1
0111011000     load constant 7
0010001110     a ex c[x]
0111010000     p + 1 -> p
0110011000     load constant 6
1100001110     if a >= c
0010001101     goyes 141d
0000001011     go to 2       error gtle or pointer increment
0111010000     p + 1 -> p
1001000010     a - 1 -> a[p]
1100001110     if a >= c     now is a[p] = c[p] (5)
0010010010     goyes 146d
0000001011     go to 2       error gtle or pointer increment
1001000010     a - 1 -> a[p] now is a less than c (4 < 6)
1100001110     if a >= c
0000000010     goyes 2d      error in gtle

0000010011     go to 4           end
