import std.stdio;

struct TestStruct{
    float /*------------------*/ a;
    float /*------------------*/ b;
    float function( TestStruct ) op;

    float run(){
        return op( this );
    }
}

void main(){
    TestStruct ts = {
        a: 2, b: 3,
        op: (s) {
            return s.a + s.b;
        }
    };

    writeln( ts.op(ts) );
    writeln( ts.run()  );
}