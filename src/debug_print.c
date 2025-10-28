#include <ultra64.h>
#include <string.h>
#include "res/debug_font.h"

void Debug_Print(Gfx **glistp_ptr, u16 x, u16 y, const char *str)
{
	char sbuf[128];
    Gfx *glistp = *glistp_ptr;
    int i;

	sprintf(sbuf, "%s", str);

    gDPSetCycleType(glistp++, G_CYC_1CYCLE);
    gDPSetCombineMode(glistp++, G_CC_DECALRGBA, G_CC_DECALRGBA);
    gDPSetRenderMode(glistp++, G_RM_NOOP, G_RM_NOOP2);
    gDPSetTexturePersp(glistp++, G_TP_NONE);

    // Load once, not per character
    gDPLoadTextureBlock_4b(glistp++, debug_font, G_IM_FMT_IA,
                           128, 64, 0,
                           G_TX_WRAP, G_TX_WRAP,
                           G_TX_NOMASK, G_TX_NOMASK,
                           G_TX_NOLOD, G_TX_NOLOD);

    for (i = 0; i < strlen(sbuf); i++)
    {
        int print_x = x + (i * 8);
        int de_ascii = sbuf[i] - 32;
        int font_x = (de_ascii % 16) * 8;
        int font_y = (de_ascii / 16) * 16;

        gSPTextureRectangle(glistp++,
            print_x << 2, y << 2,
            (print_x + 8) << 2, (y + 16) << 2,
            G_TX_RENDERTILE,
            font_x << 5, font_y << 5,
            1 << 10, 1 << 10);
    }

    gDPPipeSync(glistp++);
    *glistp_ptr = glistp;
}
