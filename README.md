# custom_rv64im

Custom implementation of the user part of risc-v rv64im core.
All the system part is not following the risc-v System specs.
It contains a custom MMU, L1 caches, and some custom crypto instructions.

There's also a top design that instanciates two of these cores, a bus/L2 cache/interrupt controller.
And some devices.

I use it on a KU040 dev board, so it's targeted for it.

The USB controller contains a SIE and a small rv32i core to handle the logic.

All the non-included parts are vendor IPs (block ram, DDR4 controller, div/mul, ...).

