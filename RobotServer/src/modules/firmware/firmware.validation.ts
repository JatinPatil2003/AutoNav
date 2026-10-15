import { z } from "zod";

/**
 * hue/sat/val/speedMs are all-or-nothing: omit all four to let the ESP32
 * apply that pattern's hardcoded default color/speed (see
 * DefaultParamsForPattern in the firmware's led_pattern.h), or supply all
 * four to fully override them. A partial set can't be forwarded as a
 * single wire payload (see esp32_bridge_node's handleSetLedPattern), so
 * it's rejected here instead of silently filling gaps with guesses.
 */
export const ledPatternSchema = z
  .object({
    patternId: z.number().int().min(0).max(255),
    hue: z.number().int().min(0).max(255).optional(),
    sat: z.number().int().min(0).max(255).optional(),
    val: z.number().int().min(0).max(255).optional(),
    speedMs: z.number().int().min(0).max(65535).optional(),
  })
  .refine(
    (body) => {
      const provided = [body.hue, body.sat, body.val, body.speedMs].filter((v) => v !== undefined).length;
      return provided === 0 || provided === 4;
    },
    { message: "hue, sat, val, and speedMs must be given together, or all omitted to use the pattern's default" },
  );
export type LedPatternDto = z.infer<typeof ledPatternSchema>;

export const brightnessSchema = z.object({
  brightness: z.number().int().min(0).max(255),
});
export type BrightnessDto = z.infer<typeof brightnessSchema>;
