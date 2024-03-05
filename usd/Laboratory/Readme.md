## USDA texture pathing issue:
To make textures portable, when you save as .usda open it as code, in the line where you define the world, you can see you define “Laboratorio”, it has an absolute path, that won’t work if trying to open it in another computer. 
To fix this issue, remove the part on the right of the =, and substitute it with ./Laboratorio.usd .
This should fix the pathing errors for textures as long as everything is in the same folder as the .usda that you are trying to open/import.

## References to make wall X0 Y0 Z0
|             | X     | Y   | Z    |
|-------------|-------|-----|------|
| Translation | 365.5 | 20  | -170 |
| Rotation    | 0     | 0   | 0    |
| Scale       | 100   | 100 | 100  |
