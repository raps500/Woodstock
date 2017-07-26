     decimal
     gonc start
error:
     nop
     gonc error     ; used for error
success:
     nop
     gonc success
start:
     0 -> a[w]
     if 0 # a[w]     ;test clear register a
     goyes error
     0 -> c[w]
     if 0 # c[w]     ;test clear register c
     goyes error
     p <- 13
     load constant 9
     if 0 # c[w]     ;test load register c
     goyes tstload0
     if p # 12    ;test p compare for not equal
tstload0:
     goyes error
     if p = 12    ;test p compare for equal
     goyes tstload1
     gonc error
tstload1:
     load constant 9
     load constant 9
     load constant 9
     load constant 9
     load constant 9
     load constant 9
     load constant 9
     load constant 9
     load constant 9
     load constant 9
     load constant 9
     load constant 9
     load constant 9
     c -> a[w]
     a -> b[w]
     rotate a left
     if 0 # a[w]
     goyes next0
     gonc error         ;rotate or cmp a # 0 fail
next0:
     a + 1 -> a[x]
     if 0 # a[x]
     gonc error           ;a[x] + 1 error
     a + 1 -> a[m]
     if 0 # a[m]
     gonc error           ;a[m] + 1 error
     a + 1 -> a[s]
     if 0 # a[s]
     gonc error           ;a[s] + 1 error
     c + 1 -> c[w]
     if 0 # c[w]
     gonc error         ;c[w] + 1 error
     0 -> c[w]
     p <- 13
     load constant 9  ; test registers
     c -> data register 15
     shift right c[w]
     c -> data register 14
     shift right c[w]
     c -> data register 13
     shift right c[w]
     c -> data register 12
     shift right c[w]
     c -> data register 11
     shift right c[w]
     c -> data register 10
     shift right c[w]
     c -> data register 9
     shift right c[w]
     c -> data register 8
     shift right c[w]
     c -> data register 7
     shift right c[w]
     c -> data register 6
     shift right c[w]
     c -> data register 5
     shift right c[w]
     c -> data register 4
     shift right c[w]
     c -> data register 3
     shift right c[w]
     c -> data register 2
     0 -> a[w]
     0 -> c[w]
     data register 2 -> c 
     a + c -> a[w]
     data register 3 -> c 
     a + c -> a[w]
     data register 4 -> c 
     a + c -> a[w]
     data register 5 -> c 
     a + c -> a[w]
     data register 6 -> c 
     a + c -> a[w]
     data register 7 -> c 
     a + c -> a[w]
     data register 8 -> c 
     a + c -> a[w]
     data register 9 -> c 
     a + c -> a[w]
     data register 10 -> c
     a + c -> a[w]
     data register 11 -> c
     a + c -> a[w]
     data register 12 -> c
     a + c -> a[w]
     data register 13 -> c
     a + c -> a[w]
     data register 14 -> c
     a + c -> a[w]
     data register 15 -> c
     a + c -> a[w]
     a - b -> a[w] ;use summed result to clear A
     if 0 # a[w]
     goyes error
     a + b -> a[w] ;test decimal add
     a + 1 -> a[w] ;clear a
     if 0 # a[w]
     goyes error
     0 -> c[w]
     c - 1 -> c[w]    ;test decimal sub
     a ex c[w]
     a - b -> a[w] ;use summed result to clear A
     if 0 # a[w]
     goyes error
     0 -> a[w]
     0 -> c[w]
     c - 1 -> c[w]    ;test decimal sub
     a - c -> c[w]    ;test reversed sub
     a ex c[w]
     a - 1 -> a[w]    ;test decimal sub on a
     if 0 # a[w]
     goyes error
     0 -> c[x]     ;** test pointer and cmp gtle
     p <- 1
     load constant 7
     a ex c[x]
     p + 1 -> p
     load constant 6
     if a >= c[w]
     goyes next1
     gonc error    ;   error gtle or pointer increment
next1:
     p + 1 -> p
     a - 1 -> a[p]
     if a >= c[w]     ;now is a[p] = c[p] (5)
     goyes next2
     gonc error    ;   error gtle or pointer increment
next2:
     a - 1 -> a[p] ;now is a less than c (4 < 6)
     if a >= c[w]
     goyes error      ;error in gtle

     gonc success
