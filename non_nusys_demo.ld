OUTPUT_ARCH (mips)

#define BEGIN_SEG(name, addr) \
    _##name##SegmentStart = ADDR(.name); \
    _##name##SegmentRomStart = __romPos; \
    .name addr : AT(__romPos)

#define END_SEG(name) \
    _##name##SegmentEnd = ADDR(.name) + SIZEOF(.name); \
    _##name##SegmentRomEnd = __romPos + SIZEOF(.name); \
    __romPos += SIZEOF(.name);

#define BEGIN_NOLOAD(name) \
    _##name##SegmentBssStart = ADDR(.name.noload); \
    .name.noload (NOLOAD) :

#define END_NOLOAD(name) \
    _##name##SegmentBssEnd = ADDR(.name.noload) + SIZEOF(.name.noload); \
    _##name##SegmentBssSize = SIZEOF(.name.noload);

SECTIONS
{
   __romPos = 0;

   BEGIN_SEG(boot, 0x04000000)
   {
      build/obj/rom_header.o(.text);
      build/obj/boot.6102.o(.data);
   }
   END_SEG(boot)

   BEGIN_SEG(code, 0x80000400) SUBALIGN(16)
   {
      /* text */
      build/obj/entry.o(.text);
      build/codesegment.o(.text);
      WORK_DIR/lib/n64/PR/rspboot.o(.text);
	  WORK_DIR/ucode/gspF3DEX3_BrZ_NOC.o(.text);
      /* WORK_DIR/lib/n64/PR/gspF3DEX2.fifo.o(.text); */
	  WORK_DIR/lib/n64/PR/aspMain.o(.text);
	  
      /* data */
      build/codesegment.o(.data*);
      WORK_DIR/lib/n64/PR/rspboot.o(.data*);
	  WORK_DIR/ucode/gspF3DEX3_BrZ_NOC.o(.data);
      /* WORK_DIR/lib/n64/PR/gspF3DEX2.fifo.o(.data*); */
	  WORK_DIR/lib/n64/PR/aspMain.o(.data*);
	  
      /* rodata */
      build/codesegment.o(.rodata*);
   }
   END_SEG(code)
   BEGIN_NOLOAD(code)
   {
      build/codesegment.o(COMMON);
      build/codesegment.o(.scommon*);
      build/codesegment.o(.bss*);
      . = ALIGN(0x8);
   }
   END_NOLOAD(code)
   
   BEGIN_SEG(audio_data, __romPos)
   {
      build/obj/audio_data.o(.data);
	  . = ALIGN(0x8);
   }
   END_SEG(audio_data)
   
   BEGIN_SEG(background_data, 0x02000000)
   {
	  build/obj/higu.o(.data);
	  . = ALIGN(0x8);
   }
   END_SEG(background_data)
   
   BEGIN_SEG(model_data, 0x03000000){
	  build/obj/model.inc.o(.data);
   }
   END_SEG(model_data)
   
   /* Discard everything not specifically mentioned above. */
   /DISCARD/ :
   {
      *(*);
   }
}
