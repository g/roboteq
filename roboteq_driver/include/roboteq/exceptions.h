#ifndef ROBOTEQ_EXCEPTIONS
#define ROBOTEQ_EXCEPTIONS

namespace roboteq {

class NoHandler : public std::exception {
};

class BadConnection : public std::exception {
};

class BadTransmission : public std::exception {
};

}

#endif
