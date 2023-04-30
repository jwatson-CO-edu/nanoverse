#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
nd_vector.h
James Watson, 2015 October , 2018 September Update
Implementing a class for a vector of arbitrary dimensions , Originally for ME-6250_Prog-for-Eng at U of Utah
Update: Can be templated on either floating point type and on dimension

Template Version: 2018-07-16
***********/
/*
DEV PLAN: The very moment this spills over into matrix operations, switch to GLM!
*/

#ifndef ND_VECTOR_H // This pattern is to prevent symbols to be loaded multiple times
#define ND_VECTOR_H // from multiple imports

// ~~ Includes ~~
// ~ Local ~
#include <Cpp_Helpers.h>

// ~~ Shortcuts and Aliases ~~

// ~~ Constants ~~


// === Classes and Structs =================================================================================================================

namespace ndv{ // NAMESPACE BEGIN

template< typename FPT , uint DIM >
class nd_vector{
private:
	uint dimension_ = DIM;
	FPT* elements_  = nullptr;
public:
	// create a default vector
	nd_vector()
		: dimension_( DIM )
		, elements_( new FPT[ dimension_ ] ) 
	{ /* Empty, default values used */ }
	
	nd_vector( FPT val )
		// if dimension = 3 and val = 2, create the vector [2, 2, 2]
		: dimension_( DIM )
		, elements_( new FPT[ dimension_ ] ) 
	{ // Assign the default value 'val' to each of the members
		for( uint i = 0 ; i < dimension_ ; ++i ){
			elements_[i] = val;
		}
	}
	
	nd_vector( FPT val1 , FPT val2  )
		: dimension_( DIM )
		, elements_( new FPT[ dimension_ ] ) 
	{ 
		assert( DIM == 2 ); // check valid dimension
		elements_[0] = val1;
		elements_[1] = val2;
	}
	
	nd_vector( FPT val1 , FPT val2 , FPT val3 )
		: dimension_( DIM )
		, elements_( new FPT[ dimension_ ] ) 
	{ 
		assert( DIM == 3 ); // check valid dimension
		elements_[0] = val1;
		elements_[1] = val2;
		elements_[2] = val3;
	}
	
	nd_vector( const nd_vector<FPT,2>& dim2 , FPT valZ )
		: dimension_( DIM )
		, elements_( new FPT[ dimension_ ] ) 
	{ 
		assert( DIM == 3 ); // check valid dimension
		elements_[0] = dim2[0];
		elements_[1] = dim2[1];
		elements_[2] = valZ;
	}

	// copy constructor, create an nd_vector from another
	nd_vector( const nd_vector<FPT,DIM>& other )
		: dimension_( DIM )
		, elements_( new FPT[ DIM ] )
	{
		for( uint i = 0 ; i < dimension_ ; ++i ){
			elements_[i] = other.elements_[i]; // deep copy
		}
	}

	// assignment operator
	nd_vector<FPT,DIM>& operator=( const nd_vector<FPT,DIM>& other ){
		if( this != &other ) { // pointless to assign to itself
			dimension_ = other.dimension_; // same dimension
			delete[] elements_; // delete array of old values
			elements_ = new FPT[dimension_]; // create arr of appropriate dim
			for( uint i = 0 ; i < dimension_ ; ++i ){
				elements_[i] = other.elements_[i]; // deep copy
			}
		}
		return *this;
	}

	// de-allocate any allocated memory
	~nd_vector(){ // destructor
		delete[] elements_; // delete array of old values 
		dimension_ = 0; // just in case
	}

	// return the dimension of the vector
	uint dimension() const { return dimension_; }

	// return a const reference to the ith element of the vector
	const FPT& operator[](uint i) const {
		assert(  i >= 0  &&  i < dimension()  ); // check valid index
		return elements_[i];
	}

	// return a reference to the ith element of the vector
	FPT& operator[](uint i){
		assert(  i >= 0  &&  i < dimension()  ); // check valid index
		return elements_[i];
	}
	
	// Load the elems into a normal array
	void load_array( FPT (&outArr)[DIM] ){  
		for( uint i = 0 ; i < DIM ; i++ ){  outArr[i] = elements_[i];  }
	}
	
	// http://courses.cms.caltech.edu/cs11/material/cpp/donnie/cpp-ops.html
	
	nd_vector<FPT,DIM>& operator+=( const nd_vector<FPT,DIM>& rhs ){
		for( uint i = 0 ; i < DIM ; i++ ){  elements_[i] += rhs[i];  }
		return *this;
	}
	
	// Add this instance's value to other, and return a new instance
	// with the result.
	const nd_vector<FPT,DIM> operator+( const nd_vector<FPT,DIM>& other ) const {
		nd_vector<FPT,DIM> result = *this;     // Make a copy of myself.  Same as MyClass result(*this);
		result += other;            // Use += to add other to the copy.
		return result;              // All done!
	}
};

// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

//~ // return u + v
//~ template< typename FPT , uint DIM >
//~ nd_vector<FPT,DIM> operator+( const nd_vector<FPT,DIM>& u, const nd_vector<FPT,DIM>& v) { // elem-wise add
	//~ // assert( u.dimension() == v.dimension() );
	//~ nd_vector<FPT,DIM> result( u.dimension() , 0 );
	//~ for( uint i = 0 ; i < u.dimension() ; ++i ){ // add by elems
		//~ result[i] = u[i] + v[i];
	//~ }
	//~ return result;
//~ }

//~ template< typename FPT , uint DIM >
//~ nd_vector<FPT,DIM> operator+( const nd_vector<FPT,DIM>& u, nd_vector<FPT,DIM>& v) { // elem-wise add
	//~ // assert( u.dimension() == v.dimension() );
	//~ nd_vector<FPT,DIM> result( );
	//~ for( uint i = 0 ; i < u.dimension() ; ++i ){ // add by elems
		//~ result[i] = u[i] + v[i];
	//~ }
	//~ return result;
//~ }

// return u - v
template< typename FPT , uint DIM >
nd_vector<FPT,DIM> operator-( const nd_vector<FPT,DIM>& u , const nd_vector<FPT,DIM>& v ){ 
	//~ assert( u.dimension() == v.dimension() );
	nd_vector<FPT,DIM> result(  );
	for( uint i = 0 ; i < u.dimension() ; ++i ){ // subtract by elems
		result[i] = u[i] - v[i];
	}
	return result;
}

// return u * v (element wise)
template< typename FPT , uint DIM >
nd_vector<FPT,DIM> operator*( const nd_vector<FPT,DIM>& u , const nd_vector<FPT,DIM>& v ){
	//~ assert( u.dimension() == v.dimension() );
	nd_vector<FPT,DIM> result( u.dimension() , 0 );
	for( uint i = 0 ; i < u.dimension() ; ++i ){ // mult by elems
		result[i] = u[i] * v[i];
	}
	return result;
}

// return u / v (element wise)
template< typename FPT , uint DIM >
nd_vector<FPT,DIM> operator/( const nd_vector<FPT,DIM>& u , const nd_vector<FPT,DIM>& v ){ 
	//~ assert( u.dimension() == v.dimension() );
	nd_vector<FPT,DIM> result( u.dimension() , 0 );
	for( uint i = 0 ; i < u.dimension() ; ++i ){ // div by elems
		result[i] = u[i] / v[i];
	}
	return result;
}

// return u * x // scale the vector by a constant
template< typename FPT , uint DIM >
nd_vector<FPT,DIM> operator*( const nd_vector<FPT,DIM>& u , FPT x ){
	nd_vector<FPT,DIM> result( u.dimension() , 0 );
	for( uint i = 0 ; i < u.dimension(); ++i) { // scale each elem
		result[i] = u[i] * x;
	}
	return result;
}

// return x * u
template< typename FPT , uint DIM >
nd_vector<FPT,DIM> operator*( FPT x , const nd_vector<FPT,DIM>& u ){ // above with reversed params
	nd_vector<FPT,DIM> result( u.dimension() , 0 );
	for( uint i = 0 ; i < u.dimension() ; ++i ){
		result[i] = u[i] * x;
	}
	return result;
}

// return u / x
template< typename FPT , uint DIM >
nd_vector<FPT,DIM> operator/( const nd_vector<FPT,DIM>& u , FPT x ){ // divide by scalar
	nd_vector<FPT,DIM> result( (FPT)0.0 );
	for( uint i = 0 ; i < u.dimension() ; ++i ){ // divide elems by scalar
		result[i] = u[i] / x;
	}
	return result;
}

// u = u + v
template< typename FPT , uint DIM >
nd_vector<FPT,DIM>& operator+=( nd_vector<FPT,DIM>& u , const nd_vector<FPT,DIM>& v ){
	//~ assert(u.dimension() == v.dimension());
	for (int i = 0; i < u.dimension(); ++i) { // add by elems
		u[i] += v[i];
	}
	return u;
}

// u = u - v
template< typename FPT , uint DIM >
nd_vector<FPT,DIM>& operator-=( nd_vector<FPT,DIM>& u , const nd_vector<FPT,DIM>& v ){
	//~ assert( u.dimension() == v.dimension() );
	for( uint i = 0 ; i < u.dimension() ; ++i ){ // minus-assign each elem v from u
		u[i] -= v[i];
	}
	return u;
}

// u = u * v
template< typename FPT , uint DIM >
nd_vector<FPT,DIM>& operator*=( nd_vector<FPT,DIM>& u , const nd_vector<FPT,DIM>& v ){
	//~ assert( u.dimension() == v.dimension() );
	for( uint i = 0 ; i < u.dimension() ; ++i ){
		u[i] *= v[i];
	}
	return u;
}

// u = u / v
template< typename FPT , uint DIM >
nd_vector<FPT,DIM>& operator/=( nd_vector<FPT,DIM>& u , const nd_vector<FPT,DIM>& v ){ 
	//~ assert( u.dimension() == v.dimension() );
	for( uint i = 0 ; i < u.dimension() ; ++i ){ // divide-assign elems u by v
		u[i] /= v[i];
	}
	return u;
}

// u = u * x
template< typename FPT , uint DIM >
nd_vector<FPT,DIM>& operator*=( nd_vector<FPT,DIM>& u , FPT x ){
	for (int i = 0; i < u.dimension(); ++i) {
		u[i] *= x;
	}
	return u;
}

// u = u / x
template< typename FPT , uint DIM >
nd_vector<FPT,DIM>& operator/=( nd_vector<FPT,DIM>& u , FPT x ){ 
	for( uint i = 0 ; i < u.dimension() ; ++i ){ // minus-assign elems by scalar
		u[i] /= x;
	}
	return u;
}

// return -u
template< typename FPT , uint DIM >
nd_vector<FPT,DIM> operator-( const nd_vector<FPT,DIM>& u ){ return u * -1.0; }

// return u == v
template< typename FPT , uint DIM >
bool operator==( const nd_vector<FPT,DIM>& u, const nd_vector<FPT,DIM>& v ){ 
	bool isEqual = true; // eq flag
	for( uint i = 0 ; i < u.dimension() ; ++i ){
		// if we found a pair of correspong elems unequal, set flag false, stop comparison
		if (u[i] != v[i]) { isEqual = false; break; }
	}
	return isEqual;
}

// return u != v
template< typename FPT , uint DIM >
bool operator!=( const nd_vector<FPT,DIM>& u , const nd_vector<FPT,DIM>& v){ return !(u == v); }

// dot product
template< typename FPT , uint DIM >
FPT dot( const nd_vector<FPT,DIM>& u , const nd_vector<FPT,DIM>& v ){ 
	FPT rtnDot = 0.0;
	for( uint i = 0 ; i < u.dimension() ; i++ ){ // for each pair of elems, mult and add to total
		rtnDot += u[i] * v[i];
	}
	return rtnDot;
}
// 2-norm (e.g. norm([3, 4]) = sqrt(3 * 3 + 4 * 4))

template< typename FPT , uint DIM >
FPT norm( const nd_vector<FPT,DIM>& u ){ // norm of vector
	return sqrt( dot( u , u ) );
}

// return the normalized vector
template< typename FPT , uint DIM >
nd_vector<FPT,DIM> normalized( const nd_vector<FPT,DIM>& u ){
	FPT l = norm( u );
	return l > 0 ? (u / l) : u;
}

// if u and v are 3D vectors, return the cross product u × v
template< typename FPT >
nd_vector<FPT,3> cross( const nd_vector<FPT,3>& u , const nd_vector<FPT,3>& v ){
	// NOTE: Cross product is technically defined for any 1-dimensional vector, just not here!
	//~ assert( u.dimension() == 3 && v.dimension() == 3 );
	nd_vector<FPT,3> rtnCross = nd_vector<FPT,3>( 3 , 0.0 );
	// using cofactor expansion
	rtnCross[0] = u[1] * v[2] - u[2] * v[1];
	rtnCross[1] = u[2] * v[0] - u[0] * v[2];
	rtnCross[2] = u[0] * v[1] - u[1] * v[0];
	return rtnCross;
}

// return true if u and v are orthogonal (i.e. dot(u, v) == 0)
template< typename FPT , uint DIM >
bool is_orthogonal( const nd_vector<FPT,DIM>& u , const nd_vector<FPT,DIM>& v ){ 
	return dot(u, v) == 0;
}

// write u to an output stream
template< typename FPT , uint DIM >
std::ostream& operator<<( std::ostream& output , const nd_vector<FPT,DIM>& u ){
	output << "("; // open paren
	for (uint i = 0; i < u.dimension(); i++) { // for each elem
		output << u[i];
		if(i < u.dimension() - 1){ output << " "; } // trailing space for all but last
	}
	output << ")"; // close paren
	return output;
}

// read u from an input stream
template< typename FPT , uint DIM >
std::istream& operator>>( std::istream& is , nd_vector<FPT,DIM>& u ){
	is.ignore( std::numeric_limits<std::streamsize>::max() , '(' );
	std::vector<FPT> elems;
	while( is.peek() != ')' ){
		FPT e = 0;
		is >> e;
		elems.push_back(e);
	}
	char c = static_cast<char>( is.get() );
	assert( c == ')' );
	u = nd_vector<FPT,DIM>( static_cast<int>(elems.size()) , 0 );
	for( uint i = 0 ; i < u.dimension() ; ++i ){
		u[i] = elems[i];
	}
	return is;
}

template< typename FPT , uint DIM >
bool eqf0( const nd_vector<FPT,DIM>& v ){
	for( uint i = 0 ; i < DIM ; i++ ){ // for each pair of elems, mult and add to total
		if( !eqf( v[i] , 0.0f ) ){  return false;  }
	}
	return true;
}

template< typename FPT , uint DIM >
FPT vec_proj( const nd_vector<FPT,DIM>& a , const nd_vector<FPT,DIM>& b ){
    // 'a' projected onto 'b', 'a' scalar length
    return dot( a , b ) / norm( b ); // Note that the result will be negative if the angle between a and b is > pi/2
}

template< typename FPT > // FIXME: Projection could technically apply to an n-dimensional plane
nd_vector<FPT,3> vec_proj_to_plane( nd_vector<FPT,3> vec , nd_vector<FPT,3> planeNorm ){
    // Return a vector that is the projection of 'vec' onto a plane with the normal vector 'planeNorm'
    // URL, projection of a vector onto a plane: http://www.euclideanspace.com/maths/geometry/elements/plane/lineOnPlane/
    // NOTE: This function assumes 'vec' and 'planeNorm' are expressed in the same bases
    if( eqf0( vec ) ){  return nd_vector<FPT,3>{ 0 , 0 , 0 };  }
    else{
        nd_vector<FPT,3> projDir = normalized( cross( cross( planeNorm , vec ) , planeNorm ) ); // direction of the projected vector, normalize
        FPT projMag = vec_proj( vec , projDir ); // magnitude of the vector in the projected direction
        return projDir * projMag; // scale the unit direction vector to the calculated magnitude and return
	}
}

template< typename FPT >
nd_vector<FPT,3> rand_3(){
	nd_vector<FPT,3> rtnVec; 
	for( uint i = 0 ; i < 3 ; i++ ){  
		rtnVec[i] = randrange( (FPT)0.0 , (FPT)1.0 );  
	}
	return rtnVec;
}

// ___ End Func ____________________________________________________________________________________________________________________________

} // NAMESPACE END

#endif

/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/
