/*H**************************************************************************
* FILENAME:	base_init.c
*
* DESCRIPTION:
*	Hyrax and hyrax-derivative board specific initialisation.
*
* NOTES:
*	Copyright (c) IntelliDesign, 2016.  All rights reserved.
*
* CHANGES:
*
* VERS-NO CR-NO     DATE    WHO	DETAIL
*	1				21Nov16	WTO	Created
*
*H*/

/***********************************************************************
*	INCLUDE FILES
***********************************************************************/

/*----- system files -------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

/***********************************************************************
*	DECLARATIONS
***********************************************************************/

/*----- context ------------------------------------------------------*/

#define DRIVER_AUTHOR "<will@intellidesign.com.au>"
#define DRIVER_DESC "Hyrax Instantiation"

#define PCP105					26819
#define FIT_MACHINE				28823
#define PCP105_CATERPILLAR		29655
#define NEC						30602

#define HWREV_P1                0x91
#define HWREV_P2                0x92
#define HWREV_A	                0xA0

/*----- prototypes ---------------------------------------------------*/

int pcp105p1_init(void);
int pcp105p2_init(void);
int fitp1_init(void);
int fita_init(void);
int necp1_init(void);

/*----- variables ----------------------------------------------------*/

int pca_id = 0;
int pca_revision = 0;

module_param (pca_id, uint, 0644);
module_param (pca_revision, uint, 0644);

/***********************************************************************
*	FUNCTIONS
***********************************************************************/

/*F*********************************************************************
* NAME: static char *IdString( int id )
*
* DESCRIPTION:
*	Returns a product string based on the given ID
*
* INPUTS:
*	id				product ID.
*
* OUTPUTS:
*	char*			product string.
*
* NOTES:
*
*F*/

static char *IdString( int id )
{
	static char zString[32] = "";

	switch ( id )
	{
	case ( PCP105 ) :
	case ( PCP105_CATERPILLAR ) :
		strcpy( zString, "PCP105" );
		break;
	case ( FIT_MACHINE ) :
		strcpy( zString, "Fit" );
		break;
	case ( NEC ) :
		strcpy( zString, "NEC" );
		break;
	default :
		strcpy( zString, "Unknown" );
		break;
	}
	return ( zString );
}

/*F*********************************************************************
* NAME: static char *RevisionString( int iRevision )
*
* DESCRIPTION:
*	returns a h/w version code based on an integer value.
*
* INPUTS:
*	iRevision		integer value
*
* OUTPUTS:
*	char*			version code.
*
* NOTES:
*
*F*/

static char *RevisionString( int iRevision )
{
	static char zString[32] = "";

	if ( ( iRevision >= 0x91 ) && ( iRevision < 0xa0 ) )
	{
		sprintf( zString, "P%d", iRevision-0x90 );
	}
	else if ( ( iRevision >= 0xa0 ) && ( iRevision <= (0xa0+('Z'-'A')) ) )
	{
		sprintf( zString, "%c", (iRevision-0xa0)+'A' );
	}
	else
	{
		strcpy( zString, "Unknown" );
	}
	return ( zString );
}

/*F*********************************************************************
* NAME: static int hyrax_init(void) 
*
* DESCRIPTION:
*	Base initialisation code.
*
* INPUTS:
*	None.
*
* OUTPUTS:
*	None.
*
* NOTES:
*
*F*/

static int hyrax_init(void) 
{  
	printk( "%s PCAId %d(%s), rev %Xh(%s)\n", __FUNCTION__, 
			pca_id, IdString(pca_id), 
			pca_revision, RevisionString(pca_revision) );

	switch ( pca_id )
	{

/*----- Handle PCP105 ------------------------------------------------*/	
	default :
	case ( PCP105 ) :
	case ( PCP105_CATERPILLAR ) :
		switch ( pca_revision )
		{
		case ( HWREV_P1 ) :
			pcp105p1_init();
			break;
		default :
		case ( HWREV_P2 ) :
			pcp105p2_init();
			break;
		}
		break;

/*----- Handle Movus router ------------------------------------------*/	
	case ( FIT_MACHINE ) :
		switch ( pca_revision )
		{
		case ( HWREV_P1 ) :
			fitp1_init();
			break;
		default :
		case ( HWREV_A ) :
			fita_init();
			break;
		}
		break;

/*----- Handle NEC board ---------------------------------------------*/	
	case ( NEC ) :
		switch ( pca_revision )
		{
		default :
		case ( HWREV_P1 ) :
			necp1_init();
			break;
		}
		break;
	}

	return 0;
}  

static void __exit hyrax_exit(void)
{
}

//module_init(hyrax_init);
arch_initcall(hyrax_init);
module_exit(hyrax_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("Dual BSD/GPL");

