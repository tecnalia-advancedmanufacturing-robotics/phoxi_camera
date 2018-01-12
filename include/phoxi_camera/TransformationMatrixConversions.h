//
// Created by controller on 1/12/18.
//

#ifndef PROJECT_TRANSFORMATIONMATRIXCONVERSIONS_H
#define PROJECT_TRANSFORMATIONMATRIXCONVERSIONS_H

#include <phoxi_camera/CoordinatesSpaceTypes.h>
#include <phoxi_camera/TransformationMatrix.h>
#include <Eigen>

namespace phoxi_camera{
    phoxi_camera::TransformationMatrix transformationMatrixToMsg(Eigen::Matrix4d t, phoxi_camera::CoordinatesSpace space){
        phoxi_camera::TransformationMatrix msg;
        msg.request.coordinates_space = space;
        for(int i = 0; i < 4 ; i++){
            for(int j = 0; j < 4 ; j++){
                msg.request.matrix.push_back(t(i,j));
            }
        }
        return msg;
    }
}
#endif //PROJECT_TRANSFORMATIONMATRIXCONVERSIONS_H
