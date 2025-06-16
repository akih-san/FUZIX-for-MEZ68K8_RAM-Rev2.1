OUTPUT_ARCH(m68k)

SEARCH_DIR(.)

MEMORY
{
  ram (rwx) : ORIGIN = 0x00000, LENGTH = 0x80000
}

SECTIONS
{
  .text :
  {
    CREATE_OBJECT_SYMBOLS
    *(.text .text.*)

  } > ram

  .data :
  {
    _data = .;
    *(.got.plt) *(.got)
    *(.shdata)
    *(.data .data.*)
    _edata = .;
  } > ram

  .bss :
  {
    . = ALIGN(0x4);
    __bss_start = . ;
    *(.shbss)
    *(.bss .bss.*)
    *(COMMON)
    _end =  ALIGN (0x8);
    __end = _end;
  } > ram

  .stab 0 (NOLOAD) :
  {
    *(.stab)
  }

  .stabstr 0 (NOLOAD) :
  {
    *(.stabstr)
  }
}
