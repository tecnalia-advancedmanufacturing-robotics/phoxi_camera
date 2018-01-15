//
// Created by controller on 1/12/18.
//

#ifndef PROJECT_PHOXIEXCEPTION_H
#define PROJECT_PHOXIEXCEPTION_H

#include <exception>
class PhoXiInterfaceException : std::exception {
public:
    PhoXiInterfaceException(std::string message) : message(message){
    }
    virtual const char* what() const throw()
    {
        return message.c_str();
    }
private:
    std::string message;
};

class PhoXiControllNotRunning : PhoXiInterfaceException {
public:
    PhoXiControllNotRunning(std::string message) : PhoXiInterfaceException(message){
    }
};

class PhoXiScannerNotFound : PhoXiInterfaceException {
public:
    PhoXiScannerNotFound(std::string message) : PhoXiInterfaceException(message){
    }
};
class UnableToConnect : PhoXiInterfaceException {
public:
    UnableToConnect(std::string message) : PhoXiInterfaceException(message){
    }
};

class CorruptedFrame : PhoXiInterfaceException {
public:
    CorruptedFrame(std::string message) : PhoXiInterfaceException(message){
    }
};

class  PhoXiScannerNotConnected : PhoXiInterfaceException {
public:
    PhoXiScannerNotConnected(std::string message) : PhoXiInterfaceException(message){
    }
};

class  CoordinationSpaceNotSupported : PhoXiInterfaceException {
    public:
    CoordinationSpaceNotSupported(std::string message) : PhoXiInterfaceException(message){
    }
};


#endif //PROJECT_PHOXIEXCEPTION_H
