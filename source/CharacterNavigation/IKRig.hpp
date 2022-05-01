#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP


namespace Mona{

    class IKRig{
        public:
            IKRig();


    };

    class IKNode{
        private:
            friend class IKRig;
            IKNode();


    };

}


#endif