# FDMFC

This is a post-processing script experiment to evaluate the viability of
splitting outer walls vertically and stacking different filaments with
different variable layer heights to create the illusion of color mixing.

## Idea

If you want a light gray wall at 0.2mm layer height, you'd first draw
a 0.05mm wall with black filament and then on top a 0.15mm wall with
white filament.

Ideally this would also expand to CMYKW or similar for full color printing.

The biggest difficulty is probably going to be precisely extruding the amount
of material needed for a fraction of the total color, especially for small
fractions. Maybe this could be improved by "overextruding" but setting back
the material further into the wall so that it gets partially covered by the
next layer?
