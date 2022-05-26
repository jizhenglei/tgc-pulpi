# DBT-RISE-TGFS
Core of an instruction set simulator based on DBT-RISE implementing Minres The Good Folks Series cores. The project is hosted at https://git.minres.com/DBT-RISE/DBT-RISE-TGFS .

This repo contains only the code of the RISC-V ISS and can only be used with the DBT_RISE. A complete VP using this ISS can be found at https://git.minres.com/VP/Ecosystem-VP ~~which models SiFives FE310 controlling a brushless DC (BLDC) motor~~.

This library provide the infrastructure to build RISC-V ISS. Currently part of the library are the following implementations adhering to version 2.2 of the 'The RISC-V Instruction Set Manual Volume I: User-Level ISA':

* RV32I 	(TGF-B)
* RV32MIC	(TGF-C)

All pass the respective compliance tests. Along with those ISA implementations there is a wrapper (riscv_hart_m_p.h) implementing the Machine privileged mode as of privileged spec 1.10. The main.cpp in src allows to build a stand-alone ISS when integrated into a top-level project. For further information please have a look at [https://git.minres.com/VP/RISCV-VP](https://git.minres.com/VP/RISCV-VP).

Last but not least an SystemC wrapper is provided which allows easy integration into SystemC based virtual platforms.

Since DBT-RISE uses a generative approach other needed combinations or custom extension can be generated. For further information please contact [info@minres.com](mailto:info@minres.com).

