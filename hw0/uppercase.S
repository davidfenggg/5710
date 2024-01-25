        .section .data
input_string:   .asciz "Hello world!"

        # tohost/fromhost are data locations used by Spike
        .align 8
tohost: .byte 0
        .align 8
fromhost: .byte 0

        
        .section .text
        .globl _start
_start:
        # Load the address of the input string into a0
        la a0, input_string
        # load 97 and 123 into registers beacuse they will be used for comparisons
        li t2, 97
        li t3, 123
        # Your code here!
process_character: 
        # load the character into register t1
        lb t1, 0(a0)
        # check null; if null, go to end program
        beqz t1, end_program
        # check whether the letter is lowercase; if so, move onto next character
        blt t1, t2, next_character
        bge t1, t3, next_character
        # character should be lowercase letter; transform into upper case with offset 32
        addi t1, t1, -32
        sb t1, 0(a0)
        j next_character
            
next_character:
    addi a0, a0, 1
    j process_character 

end_program:
        # jump here when you're done
        j end_program

