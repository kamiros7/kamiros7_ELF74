  SECTION .text : CODE(2)
  THUMB
  
  EXPORT EightBitHistogram
  
  PUBLIC EightBitHistogram

EightBitHistogram
  MUL R0, R1
  MOV R4, #65536
  CMP R0, R4
  
  BLT calc_histogram
  MOV R0, #0
  BX LR
  
calc_histogram
  MOV R4, #0
  MOV R5, #0
  MOV R6, R3
 
loop_init_histogram
  STRH R4, [R6], #1
  ADD R5, #1
  CMP R5, #255
  BLS loop_init_histogram
  
  MOV R4, R2    //R4 recebe o endereco inicial da imagem   
  MOV R5, #0    //iterador do loop
  
loop_calc_histogram 
  LDRB R6, [R4], #1     // R6 recebe o valor de um pixel da imagem
  
  MOV R7, #0            // Limpa o registrador para realizar a op abaixo
  ADD R7, R6, R3        // R7 <- R6 (valor pixel) + R3 (endereco inicial do histograma)
  
  LDRH R8, [R7]         // R8 <- [R7] (Recebe o valor do histograma para determinado numero 0-255)
  ADD R8, #1            // R8 ++
  STRH R8, [R7]         // Atualiza o histograma
  
  ADD R5, #1                    // i++
  CMP R5, R0                    // Se i < image_size
  BCC loop_calc_histogram       // Se i < image_size: continua o loop
  
  BX LR
  
                B       .


                END