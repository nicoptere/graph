
function PRNG(){}

PRNG.initialize_generator = function( seed )
{
    // Create a length 624 array to store the state of the generator
    this.MT = new Uint32Array( 624 );
    this.index = 0;
    this.MT[ 0 ] = seed;
    for( i = 1 ; i < 624; i++ )
    {
        this.MT[ i ] = ( 1812433253 * ( this.MT[i-1] ^ ( this.MT[ i-1 ] >> 30  ) + i ) );
    }
};

PRNG.extract_number = function()
{
    if( this.index == 0 )
    {
        this.generate_numbers()
    }

    var y = this.MT[ this.index ];
    y = y ^ ( y >> 11 );
    y = y ^ ( ( y << 7 ) & 2636928640 );
    y = y ^ ( ( y << 15 ) & 4022730752 );
    y = y ^ ( y >> 18 );

    this.index = ( this.index + 1 ) % 624;
    return y;
};

PRNG.generate_numbers = function()
{
    for( var i = 0; i < 624; i++ )
    {
        var y = ( this.MT[ i ] & 0x80000000 ) + ( this.MT[( i + 1 ) % 624 ] & 0x7fffffff );
        this.MT[ i ] = this.MT[ ( i + 397 ) % 624] ^ ( y >> 1 );
        if ( y % 2 != 0 )
        {
            this.MT[ i ] = this.MT[ i ] ^ 2567483615;
        }
    }
};

PRNG.random = function()
{
    return ( this.extract_number() / 0x7FFFFFFF );
};