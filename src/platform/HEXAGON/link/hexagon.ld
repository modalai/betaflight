
SECTIONS
{
  .dynsym :
  {
    *(.dynsym)
  }
  .dynstr :
  {
    *(.dynstr)
  }
  .hash :
  {
    *(.hash)
  }
  .rela.dyn :
  {
    *(.rela.dyn)
  }
  .rela.plt :
  {
    *(.rela.plt)
  }
  .plt :
  {
    *(.plt)
  }
  .init :
  {
    KEEP (*(.init))
  }
  .text :
  {
    *(.text)
    *(.text.*)
    *(.rodata)
    *(.rodata.*)
  }
  .dynamic :
  {
    *(.dynamic)
  }
  .pg_registry : ALIGN(8)
  {
    PROVIDE_HIDDEN (__pg_registry_start = . );
    PROVIDE_HIDDEN (___pg_registry_start = . );
    KEEP (*(.pg_registry))
    KEEP (*(SORT(.pg_registry.*)))
    PROVIDE_HIDDEN (__pg_registry_end = . );
    PROVIDE_HIDDEN (___pg_registry_end = . );

    PROVIDE_HIDDEN (__pg_resetdata_start = . );
    PROVIDE_HIDDEN (___pg_resetdata_start = . );
    KEEP (*(.pg_resetdata))
    PROVIDE_HIDDEN (__pg_resetdata_end = . );
    PROVIDE_HIDDEN (___pg_resetdata_end = . );
  }
  .ctors :
  {
    KEEP (*(.ctors))
    KEEP (*(SORT(.ctors.*)))
  }
  .dtors :
  {
    KEEP (*(.dtors))
    KEEP (*(SORT(.dtors.*)))
  }
  .data :
  {
    *(.data)
    *(.data.*)
  }
  .got :
  {
    *(.got)
  }
  .got.plt :
  {
    *(.got.plt)
  }
  .bss :
  {
    *(.bss)
    *(.bss.*)
    *(COMMON)
  }
}
