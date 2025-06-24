# ptFlex PCB
The ptFlex PCB has been designed for manufacture from JLC-PCB. The design rules checker have been tuned for their manufacturing tolerances.

![3D rendering of the ptFlex circuit board.](/assets/pcb-board-rendering.png)

The design and update process includes updating the Schematic in KiCad, updating the PCB layout based on the schematic changes in KiCad, then output the Gerber files and assembly documents for upload to JLC-PCB.

## Schematic Design
Any changes to the schematic should be completed first. When placing a part, be sure to update the Bill of Materials information by including the following two Fields in each component's properties.

* **Digikey** - The Digikey part number for ordering parts (critical for any parts that are not auto-assembled by JLC), and import for all other parts in the event that someone would want to hand-assemble their tracker.
* **LCSC** - The JLC-PCB Part number for automated assembly. This value is not populated for through-hole parts and the SA818V transmitter module (sources from Alibaba, etc)

### Schematic Outputs

#### Bill of Materials
Once complete with the Schematic, run the Tools-Generate Bill of Materials which places the BOM inside of the /Current folder.

#### PDF Output
Run File-Plot and output a PDF version of the schematics into the /Current folder. Be sure to rename this output to ```schematic.pdf``` in the output directory before plotting the PCB design.

## PCB Design
Inside of the Schematic editor, press F8 to update the PCB layout with any changes to the schematic. Adjust any layouts as necessary.

### Design Rule Checker
Once a layout has been laid out, run the DRC to check for problems with the board. The castelated edges normally throw a dozen or more errors. Also, there are some silkscreen issues with the coax connector at the top of the board. Any other errors should be investigated and resolved.

### Update the PCB Version
Update the Silkscreen text that indicates the version of the PCB.

### PCB Outputs

#### Drill Files
Run the Files-Fabrication Outputs-Gerber. From that screen, click the Generate Drill Files and output the .DRL files into the /Current folder. The format should be Excellon, and the units Millimeters.

#### Gerbers
Close out of the Drill files window, and click the File-Fabrication-Plot button to generate the set of Gerber files. These should also go into the /Current folder.

#### Position Files
Run File-Fabrication Outputs-Component Placement Files. Select CSV for the Format, then click Generate Position File.

In the /Current folder, find the newly created ptFlex-top-pos.csv file and open it with a text editor. The very first header line needs to be replaced with this line to align with JLC's naming scheme.

```
Designator,Val,Package,Mid X,Mid Y,Rotation,Layer
```

#### PDF Output
Run File-Plot and output a PDF version of the pcb into the /Current folder. Select options for Black & White, and to plot as a single .PDF document. Be sure to rename this output to ```pcb.pdf``` in the output directory.

## Ordering the PCBs
### Collecting the Output
Inside of the /Current directory, Zip up all of the files except the *.pdf and *.csv files. Name this .Zip file ptFlex-vX.X.X.zip, as appropriate for the board version.

## Upload to JLCPCB

Log in to your JLC-PCB account at [JLCPCB.com](https://www.jlcpcb.com/).

Upload the Gerbers to the site. For the ptFlex tracker, select the following options:
* Layers: 2
* PCB Qty: As desired
* Surface Finish: HASL (with lead)
* PCB Thickness: 1.6mm
* Castellated Holes: No
* PCB Assembly: As desired


### PCB Assembly
If PCB Assembly is desired:

* Assembly Side: Top Side 
* PCBA Qty: Select the quantity, up to the number of PCBs you selected in the first part

Select Next, then Next again.

Upload the following files:
* Add BOM: Upload the Bill of Materials.csv file.
* Add CPL: Upload the ptFlex-top-pos.csv file. Make sure that you modified the first header line as described above.

Process BOM & CPL. You will likely get an Error that certain Designators don't exist in the CPL file. That's just a warning that some parts that are located on the back of the board (SA818V and the Supercap) aren't found in the placement file. Click the Continue button if you receive this error.

Review the Bill of Materials and ensure that the part description matches the expected values. If certain parts aren't in-stock, you will either have to order them, or hand-place them once the semi-finished boards arrive.

Once the board layout appears with the components placed on it, you will need to review each of the placements of the parts. Many of the semiconductors will be rotated 90 of 180 degrees out of spec, and you will need to adjust them. The best optoin is to open the ptFlex-top-pos.csv file, and edit the rotation values inside of the file. By doing this, you will save the copy of rotations locally and re-ordering will be that much simpler and safer.

The column in question is the second-to-last field where most of the default values are 0.00000 or 180.00000.

For the ptFlex the following parts need to be tweaked in the X/Y position to fit the pads correctly:
* U1 (5V Regulator)
* J1 (Power Connector)

The following components need to be rotated:
* Q1-Q5 (transistors) - 180 degrees
* U2 (3.3V Regulator) - 90 degrees
* U3 (ATMega) - -90 degrees
* U4 (GPS) - -90 degrees
* U6 (BME280) - 90 degrees


## Create an Final Output Directory
Copy the entire contents of the /Current folder into a folder named for the PCB version. For example, /v2.0.0/ for safe keeping.




