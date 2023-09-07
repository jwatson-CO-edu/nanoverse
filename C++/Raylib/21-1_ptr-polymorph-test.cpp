#include <vector>
using std::vector;
#include <memory>
using std::shared_ptr;
#include <iostream>
using std::cout, std::endl;

class Base{ public:

    // NOTE: If this function is not declared `virtual`, 
    //       then in a container of base class pointers, the base class function will be called
    virtual void say(){ // `virtual` declaration REQUIRED in order to implement polymorphism!
        cout << "Base" << endl; // A `virtual` function can have guts, it will be called if there is no override
    }

};
typedef shared_ptr<Base> basePtr;

class A : public Base{ public:  void say(){  cout << "A" << endl;  }  };
class B : public Base{ public:  void say(){  cout << "B" << endl;  }  };

int main(){

    vector<basePtr> vec;

    vec.push_back(  basePtr( new Base{} )  );
    vec.push_back(  basePtr( new A{}    )  );
    vec.push_back(  basePtr( new B{}    )  );

    for( basePtr& thing : vec ){  thing->say();  }
    /* Output:

    Base
    A
    B

    This was the desired behavior! */

    return 0;
}