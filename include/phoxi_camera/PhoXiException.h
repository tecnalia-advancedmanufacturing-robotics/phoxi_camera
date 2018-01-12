//
// Created by controller on 1/12/18.
//

#ifndef PROJECT_PHOXIEXCEPTION_H
#define PROJECT_PHOXIEXCEPTION_H

#include <exception>

class PhoXiException : std::runtime_error {
public:
    PhoXiException(std::string message) : message(message){
    }
    virtual const char* what() const throw()
    {
        return message.c_str();
    }
private:
    std::string message;
};


#endif //PROJECT_PHOXIEXCEPTION_H
