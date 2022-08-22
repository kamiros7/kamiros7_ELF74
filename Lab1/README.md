# Lab1
Integrantes - Luis Camilo Jusiani Moreira   RA: 2063166

            - João Victor Laskoski          RA: 1906470

1) Través do terminal I/O no campo Output

2) Através do campo input contido na aba Terminal I/O e apertar enter

3)      __cplusplus = 201703 
        Corresponde a versão do C++ que o compilador utiliza.

        __DATE__ = Aug 18 2022
        Data em que o código foi compilado.

        __TIME__ = 20:22:21
        Tempo em que o código foi compilado.

        __FILE__= C:\Users\kamir\Desktop\Camilo\Faculdade\2022-2\Sistemas Embarcados\kamiros7\dr_Tiva_Template\Lab1\main.cpp
        Caminho do diretório em que se encontra o arquivo que foi compilado.

        __LINE__= 119
        Número de linhas do arquivo compilado. Lembrando que é um número antigo de linhas, foi foram inseridos linhas
        com comentários explicando as demais macros.

        __STDC__= 1
        Indica se o compilador está conforme o padrão ISO C.

        __STDC_VERSION__ = 201710
        Indica a versão do padrão ISO C que o compilador está utilizando.

        __ARM_ARCH = 7
        Essa macro especifica a versão da arquitetura, sendo um processador ARM.

        __ARM_ARCH_ISA_THUMB = 2
        Especifica o conjuntos de insntruções utilizadas pelo processador.

        __ARM_SIZEOF_MINIMAL_ENUM = 1
        É o tamanho mínimo em memória que uma variável do tipo ENUM pode possuir.

        __ARM_SIZEOF_WCHAR_T = 4
        Define o tamanho em memória da variável do tipo wchar_t: 2-> 16 bits e 4 -> 32 bits.
        
        __ARMVFP__ = 4
        Versão da arquitetura do Vector Floating-Point implementada no processador ARM.

        __CORE__ = 13
        É um número inteiro que identifica o núcleo do chip em uso.
        
4)      
        Para a soma de float e guardar a soma em uma variável, foram executados 7 comandos em assembly, contendo
        comandos para salto, leitura na memória, movimentação de dados para registradores e escrita na memória (BL, LDR, MOVS, STR respectivamente).
        
        Foi utilizada a configuração strict conformance, o qual está contido dentro de c/c++ compiler -> floating-point semantics
