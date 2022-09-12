  SECTION .text : CODE(2)
  THUMB
  
  EXPORT EightBitHistogram
  
  PUBLIC EightBitHistogram

EightBitHistogram
  PUSH {R4-R8}
  MUL R0, R1    //calculate the image size
  MOV R4, #65536        //verify if the image size is higher than limit
  CMP R0, R4
  
  BLT calc_histogram
  MOV R0, #0
  BX LR
  
calc_histogram
  MOV R4, #0
  MOV R5, #0
  MOV R6, R3
 
loop_init_histogram     //loop for init the histogram if zeros
  STRH R4, [R6], #2
  ADD R5, #1
  CMP R5, #255
  BLS loop_init_histogram
  
  MOV R4, R2    //R4 recebe o endereco inicial da imagem   
  MOV R5, #0    //iterador do loop
loop_calc_histogram     //colocar lsl dentro das instrucções que estao acessando a mem. (a mesma situação para o ADD) Posso retirar o uso o R5
  LDRB R6, [R4], #1     // R6 recebe o valor de um pixel da imagem]
  LSL R6, #1            //Transpondo de 8bits para 16bits    
  ADD R7, R6, R3        // R7 <- R6 (valor pixel) + R3 (endereco inicial do histograma)
  LDRH R8, [R7]         // R8 <- [R7] (Recebe o valor do histograma para determinado numero 0-255)
  ADD R8, #1            // R8 ++
  STRH R8, [R7]         // Atualiza o histograma
  
  ADD R5, #1                    // i++
  CMP R5, R0                    // Se i < image_size
  BCC loop_calc_histogram       // Se i < image_size: continua o loop
  
  POP {R4-R8}
  BX LR                        
  
                B       .


                END