within VehicleDynamics;

package Environments
    package Interfaces 
        partial block TyreRoadInterface 
            input Real x "x position";
            input Real y "y position";
            //input Real v "velocity";
            output Real z "Road altitude";
            output Real[3] n "Road normal";
            output Real mue "Road adhesion";
        end TyreRoadInterface;
    end Interfaces;

    block NoGraphicsRoad 
        extends Interfaces.TyreRoadInterface;
    equation 
        mue = 0.7;
        n = {0, 0, 1};
        z = 0;
    end NoGraphicsRoad;
end Environments;