/* Command line interface to magfield.c */
/* version 1.05 */
/* 2/14/00 fixed help message- dip angle (down positive), variation (E positive) */
/* 4/11/00 put shared code in magfield.c */
/* 01/31/01 Corrected fprintf formatting statements from "%lf"
 *          to "%f". ("%lf" is a scanf() spec.)
 *          Jim Seymour (jseymour@LinxNet.com
*/
/*  
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License as
** published by the Free Software Foundation; either version 2 of the
** License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful, but
** WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
** General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
**
*/

#include	<stdio.h>
#include	<stdlib.h>
#include	<math.h>
#include        "magfield.h"


int main(int argc, char *argv[])
{
  /* args are double lat_deg, double lon_deg, double h, 
                   int mm, int dd, int yy,int model */
  /* output N, E, down components of B (nTesla)
     dip angle (down positive), variation (E positive) */
double lat_deg,lon_deg,h,var;
int model,yy,mm,dd;
double field[6];

if ((argc != 8) && (argc !=7)) {
fprintf(stdout,"Usage: mag lat_deg lon_deg h mm dd yy [model]\n");
fprintf(stdout,"N latitudes, E longitudes positive degrees, h in km, mm dd yy is date\n");
fprintf(stdout,"model 1,2,3,4,5,6,7,8,9,10,11,12 <=> IGRF90,WMM85,WMM90,WMM95,IGRF95,WMM2000,IGRF2000,WMM2005,IGRF2005,WMM2010,IGRF2010,WMM2015\n");
fprintf(stdout,"Default model is WMM2015, valid 1/1/15 - 1/1/20\n");
fprintf(stdout,"Output Bx (N) By (E) Bz (down) (in nTesla) dip (degrees down positive)\n");
fprintf(stdout,"variation (degrees E positive)\n");
exit(1);
}

lat_deg=strtod(argv[1],NULL);
lon_deg=strtod(argv[2],NULL);
h=      strtod(argv[3],NULL);
mm=     (int)strtol(argv[4],NULL,10);
dd=     (int)strtol(argv[5],NULL,10);
yy=     (int)strtol(argv[6],NULL,10);
if (argc == 8){
  model=  (int)strtol(argv[7],NULL,10);
}else{
  model=12;
}


var=rad_to_deg(SGMagVar(deg_to_rad(lat_deg),deg_to_rad(lon_deg),h,
                yymmdd_to_julian_days(yy,mm,dd),model,field));

fprintf(stdout,"%6.0f %6.0f %6.0f %4.2f %4.2f \n",
  field[3],field[4],field[5],
  rad_to_deg(atan(field[5]/pow(field[3]*field[3]+field[4]*field[4],0.5))),var);

fprintf(stdout,"%4.2f \n",var);

exit(0);
}
  
  

