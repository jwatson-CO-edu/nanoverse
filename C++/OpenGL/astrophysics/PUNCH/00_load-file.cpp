#include <string>
using std::string;
#include <vector>
using std::vector;
#include <iostream>
using std::cout, std::endl;
#include <fitsio.h>

typedef vector<string> vstr;

class FITS_File { public:
    fitsfile* fptr;
    vector<char[ FLEN_CARD ]> cards;
    int /*-*/ status; 
    int /*-*/ nkeys;
    string    path;
    vstr /**/ keys;
    int /*-*/ datatype;
    int /*-*/ Naxes;
    
    void report_status( string prefix = "" ){  
        /* print any error messages */
        if( prefix.size() == 0 ){  cout << "Status: ";  }else{  cout << prefix << ": ";  }
        if( status ){  fits_report_error( stderr, status );  }  
        cout << endl;
    } 

    FITS_File( string path_ ){
        status = 0; /* MUST initialize status */
        path   = path_;
        fits_open_file( &fptr, path.c_str(), READONLY, &status );
        fits_get_hdrspace( fptr, &nkeys, NULL, &status );
        cards.reserve( nkeys );
        fits_get_img_type( fptr, &datatype, &status );
        report_status( "Image Type" );
        fits_get_img_dim( fptr, &Naxes, &status );
        report_status( "Image Dims" );
        load_keys();
    }

    void load_keys(){
        for( int ii = 1; ii <= nkeys; ii++ ){ 
            fits_read_record( fptr, ii, cards[ii-1], &status ); /* read keyword */
            printf( "key %i: %s\n", ii-1, cards[ii-1] );
        }
    }

    void close(){
        fits_close_file( fptr, &status );
        report_status( "FILE CLOSE" );
    }
};

int main( int argc, char *argv[] ){


    
}