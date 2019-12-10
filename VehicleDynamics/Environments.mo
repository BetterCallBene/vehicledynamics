within VehicleDynamics;

package Environments
    block NoGraphicsRoad 
        extends Interfaces.TyreRoadInterface;
    equation 
        mue = 0.7;
        n = {0, 0, 1};
        z = 0;
    end NoGraphicsRoad;
end Environments;