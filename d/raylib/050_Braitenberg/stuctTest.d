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
    TestStruct ts = TestStruct(
        2,
        3,
        function( TestStruct s ){ return s.a + s.b; }
    );

    writeln( ts.run() );
}