# Quadcopter CAD reference

Generated with the built-in image-generation tool. This is an explanatory image, not a scale drilling template or validated structural design.

Motor source: https://tmotorhobby.com/goods-1099.html — manufacturer drawing shows four M2 holes on a diameter 9 mm pitch circle. Hole coordinates can be (4.5,0), (0,4.5), (-4.5,0), (0,-4.5). Adjacent spacing is 6.364 mm, not 9 mm. Body diameter 17.9 mm, overall length 16.6 mm.

Current motor page: https://www.t-hobby.com/products/micro-fpv-drones-brushless-motor — prop shaft 1.5 mm. Earlier conversation advice to buy a 2 mm bore propeller was incorrect; verify the purchased motor and use the matching prop mounting system.

ESC source: https://www.speedybee.com/speedybee-f405-mini-bls-35a-20x20-stack/ — BLS 35A Mini V2 ESC envelope 35 x 35 x 5.5 mm; mounting square 20 x 20 mm; PCB holes 3.5 mm, with grommets for mounting hardware. Frame holes depend on chosen screws, not PCB hole diameter.

Proposed layout: motor centres (+/-60,+/-60) mm; adjacent spacing 120 mm; opposite diagonal 169.706 mm; four 80 mm diameter propeller keep-out circles give a 200 x 200 mm overall envelope. Proposed central electronics envelope 45 x 70 mm. Measure actual boards and connectors before committing to it. These are layout choices, not manufacturer requirements. No arm thickness or material strength is validated.

Generation prompt specification: Create a white-background, blue/orange orthographic engineering infographic titled “3-INCH QUADCOPTER • CAD DIMENSION GUIDE”, with a proposed X-frame layout, enlarged motor bottom mounting view, ESC mounting view, and measurement checklist. Blue denotes component specifications, orange proposed dimensions, gray dimensions requiring physical measurement. Use the exact dimensions documented above. Include a proposed 22 mm motor pad and nominal 2.3 mm frame clearance holes for M2, subject to printer fit tests. Measure central underside relief and permitted motor screw engagement. Include USB, antenna, IMU, battery strap, wiring and cooling clearances. Omit structural thickness recommendations. Corrections: replace motor side drawing with textual envelope; recolor proposed dimensions orange; retain exactly four ESC mounting holes.
