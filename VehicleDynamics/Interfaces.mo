within VehicleDynamics;

package Interfaces "Vehicle dynamic interfaces" 
    partial block TyreRoadInterface 
        input Real x "x position";
        input Real y "y position";
        //input Real v "velocity";
        output Real z "Road altitude";
        output Real[3] n "Road normal";
        output Real mue "Road adhesion";
    end TyreRoadInterface;
end Interfaces;