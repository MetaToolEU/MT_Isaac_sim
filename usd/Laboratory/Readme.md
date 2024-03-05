## USDA texture pathing issue:
To make textures portable, when you save as .usda open it as code, in the line where you define the world, you can see you define “Laboratorio”, it has an absolute path, that won’t work if trying to open it in another computer. 
To fix this issue, remove the part on the right of the =, and substitute it with ./Laboratorio.usd .
This should fix the pathing errors for textures as long as everything is in the same folder as the .usda that you are trying to open/import.

## References
X 365.5 Y 20 Z -170
X 0 Y 0 Z 0
X 100 Y 100 Z 100
