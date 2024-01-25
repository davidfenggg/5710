
uppercase.bin:     file format elf32-littleriscv


Disassembly of section .text:

00010074 <_start>:
   10074:	ffff2517          	auipc	a0,0xffff2
   10078:	f8c50513          	addi	a0,a0,-116 # 2000 <__DATA_BEGIN__>
   1007c:	06100393          	li	t2,97
   10080:	07b00e13          	li	t3,123

00010084 <process_character>:
   10084:	00050303          	lb	t1,0(a0)
   10088:	02030063          	beqz	t1,100a8 <end_program>
   1008c:	00734a63          	blt	t1,t2,100a0 <next_character>
   10090:	01c35863          	bge	t1,t3,100a0 <next_character>
   10094:	fe030313          	addi	t1,t1,-32
   10098:	00650023          	sb	t1,0(a0)
   1009c:	0040006f          	j	100a0 <next_character>

000100a0 <next_character>:
   100a0:	00150513          	addi	a0,a0,1
   100a4:	fe1ff06f          	j	10084 <process_character>

000100a8 <end_program>:
   100a8:	0000006f          	j	100a8 <end_program>
