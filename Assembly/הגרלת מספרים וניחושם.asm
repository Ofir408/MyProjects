; multi-segment executable file template.

data segment
    ; add your data here! 
    
    pkey db "press any key...$" 
    asktoinputdata db  "Enter 5 numbers between 1 to 9 (inclusive)",0ah,0dh,"$"
    showrandomnumbersa db " The random numbers are: " ,0ah,0dh,"$" 
    checkyourguess db " Is your guess is True or False? " ,0ah,0dh,"$" 
    gosora db "    "   ,0ah,0dh,"$"                 
  ; i need to save the registers
  ; i need to save the input in other registers and compare between them    
  
  arr db 5 dup (?)   ; set arr for 5 random numbers 
  arr2 db 5 dup (?)    ; set another arr for 5 input numbers  
  arrcheck db  5 dup (?) ; check if they equel or not. 

                                                                                
                                                                                
                                                                                
mygrapics db '                                            '   ,0ah,0dh

db '                          '   ,0ah,0dh 
db '                          '   ,0ah,0dh           
db '     | |         | |    '   ,0ah,0dh
db '  ___| |_ __ _ _ |_   '   ,0ah,0dh
db ' / __| __/  | __||  ',0ah,0dh
db ' \__ \ || (_| |  | |_   '   ,0ah,0dh
db ' |___/\__\__,_|  \__|  '   ,0ah,0dh
                        
db '               '   ,0ah,0dh
db '              '   ,0ah,0dh
     db '                          '   ,0ah,0dh  ,'$' 
                        
                        
                        
                        

 ;                        1         2
 ;              012345678901234567890   1   2
 myBoard db    '+---+---+---+---+---+',0ah,0dh
            db '|   |   |   |   |   |',0ah,0dh
            db '+---+---+---+---+---+',0ah,0dh,'$' 
            
             
  myBoard2  db '+---+---+---+---+---+',0ah,0dh
            db '|   |   |   |   |   |',0ah,0dh
            db '+---+---+---+---+---+',0ah,0dh,'$'  
            
  
  myBoard3  db '+---+---+---+---+---+',0ah,0dh
            db '|   |   |   |   |   |',0ah,0dh
            db '+---+---+---+---+---+',0ah,0dh,'$'                      
                                                                                                                                                  
ends
    
                                                                                                                                                                   


stack segment
    dw   128  dup(0)
ends

code segment
  
proc Randomnumberss

        RANDGEN:           ; generate a rand no using the system time
 
  
   mov cx,5 
   mov si,0  ; for the cells in the arr. to control in their location 
 startloop:      
 
    push cx   
    
    ; part 2 - random numbers.  
    
RANDSTART: 
   cmp cl,0
    je continue                ; while mone!=5 beacuse I want 5 random numbers 
   MOV AH, 00h  ; interrupts to get system time        
   INT 1AH      ; take number from the clock     
                
   MOV BH, 57   ; the max is 56 in ascii = 9  
   MOV AH, DL  
   CMP AH, BH   ; compare with value in  DL,      
   JA RANDSTART ; if ah  more than bh, regenerate. if not, continue... 

   MOV BH, 49   ; the min is  48 in ascii = 0 
   MOV AH, DL   
   CMP AH, BH   ; compare with value in DL
   JNA RANDSTART ; if ah less than bh, regenerate.   
  
 
  mov arr[si],dl ; because the random numbers saves in dl.  
    mov ah, 5h   ;5h output in new screen.           
    int 21h 
    inc si     ; for the arrs.
     pop cx
     
    loop startloop    
    
    ret
    
endp  Randomnumberss 

              
    
  proc drawBoard    ; draw Board .
   
   push bp
   mov bp,sp
    
    push cx ; save before the change 
    lea dx, myBoard                                            
    mov ah, 09h
    int 21h
   
    pop cx    ; Returns the value that was set before the change
    pop bp ; returns bp before the changes 
    
    ret             ; add ret  beacuse i dont want again random numbers. 
    endp drawBoard     ; end p. draw Board.     
    
     
    proc drawBoard2    ; draw Board .
    push bp
    mov bp,sp
    
    push cx
    lea dx, myBoard2
    mov ah, 09h
    int 21h
     
    pop cx
    pop bp
    ret             ; add ret  beacuse i dont want again random numbers. 
    endp drawBoard2     ; end p. draw Board.       
     
          
    proc drawBoard3    ; draw Board .
    
    push bp
    mov bp,sp
    
    push cx 
    
    lea dx, myBoard3
    mov ah, 09h
    int 21h
    
    pop cx 
    pop bp
    ret             ; Returns to the place at the beginning of the procedure 
    endp drawBoard3     ; end p. draw Board.   
    
start:
 
   
   
    
; set segment registers:
    mov ax, data
    mov ds, ax
    mov es, ax 
    
     lea dx, mygrapics   ; this bring the grapics 
    mov ah, 09h
    int 21h    
      
     
    ; add your code here
     
   
 

     ; part one - bring the input from the user . 
  
   
     
    bringtheinput: ; here i will bring  data from the user
    lea dx,asktoinputdata
    mov ah ,9 
    int 21h
          
    ; take the input and save in regi  
         ; I want to input 5 numbers . 
    mov si,0
    mov di,25 ; for the table start position.      
    mov cx,5 
    mov ax,0 
     
      call drawboard ; add numbers in new table
     startloop2: 
    
   
    
    cmp cl,0 
    je randgen 
                 ; while mone!=5 beacuse I want 5  numbers    
    push cx
    mov ah, 1
	int 21h   ;          
	
	mov arr2[si],al
	mov [myBoard2 + di],al  ; to add the input numbers to my table game. ; input saves in al . 
	 
	 lea dx, gosora
     mov ah ,9 
     int 21h  
     
	inc si 
    add di, 4
    call drawBoard2 
	
	
	  
    loop startloop2
   
      pop cx  
      
  
  
      
     call Randomnumberss ; random numbers proc. 
    ; if not:        
     
     ; save the random in dl , copy dl to arr 
  
    
     
      continue:
      
     mov cx,5 
     mov si,0   
     mov di,25  ; start position .
     

    
     
    lea dx,showrandomnumbersa
    mov ah ,9 
    int 21h  
    
    showRandomNumbers:   ; show the random numbers in new table . 
   
    
     
    mov dl,arr[si]
    mov [myBoard + di], dl  ;**dl because there the random numbers, 
    add di, 4
    inc si
   
    call drawBoard ; draw a board for the random numbers. new table after add random numbers to the table. 
        
    loop showRandomNumbers
     
  
                ; part 3 - check if equel or not .  
      
       mov si,-1 ; make sure si = -1 for arr. -1 because i add +1 in the first,before the move. 
       mov cx,5 ; for the loop .
       mov di,25     ; start beggining .
       mov ax,0 
      comparing-arrays: ;  comparing arrays 
        inc si  ; check if this is the suitble place.          
        mov al,arr[si]              
        cmp arr2[si],al       ; i cant compar memort to memory. i need oger maver in loop.  
        je equel                  ; if the numbers equel .           
        cmp arr2[si],al
        jne theynotequel  ; if they  not equel.
      
           
                                         
        equel:   ;  they equel 
     mov bx,0
     mov  bl,'T'
     mov arrcheck[si],bl   ;becuse i  cant move from memory to memory. 
     jmp endLoop
     
	theynotequel: 
	mov bx,0
	mov bl,'F'
	mov arrcheck[si], bl 
	                               
endLoop:
	               
     loop comparing-arrays 
     
     ; now i want to put T or F in another arr in board.
    
  
    mov cx,5 
    
    mov di,25
    mov si,0 
    mov dl,0
    
    lea dx,checkyourguess
    mov ah ,9 
    int 21h          
              ; show T\F
              
    startloop4:   ;loop that put T or F in the board.
    push cx ; check.........
    mov dl, arrcheck [si]         
    mov [myBoard3 + di],dl ;to add the input F\T to my table game.
    add di, 4  
    inc si    ; for the arr. 
    call drawBoard3 ; to add t or f in the other board .
	loop startloop4
	
	pop cx ; bring cx before the changes . 
		   
    lea dx, pkey                     
    mov ah, 9
    int 21h        ; output string at ds:dx
    
    ; wait for any key....    
    mov ah, 1
    int 21h
    
    mov ax, 4c00h ; exit to operating system.
    int 21h    
ends

end start ; set entry point and stop the assembler.
                