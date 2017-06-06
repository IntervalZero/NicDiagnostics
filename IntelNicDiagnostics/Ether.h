


#define	ETHERMTU	1500
#define	ETHER_ADDR_LEN		6
#define ETHER_HDRLEN		14
#define	ETHER_ADDR_LEN		6	
#define	ETHER_TYPE_LEN		2	
#define	ETHER_CRC_LEN		4	
#define	ETHER_HDR_LEN		(ETHER_ADDR_LEN*2+ETHER_TYPE_LEN)
#define	ETHER_MIN_LEN		64	// minimum frame len, including CRC 
#define	ETHER_MAX_LEN		1518	// maximum frame len, including CRC
#define	ETHER_MAX_LEN_JUMBO	9018	// max jumbo frame len, including CRC 

#define	ETHER_VLAN_ENCAP_LEN	4	/* len of 802.1Q VLAN encapsulation */

typedef unsigned __int64	u64;
typedef unsigned __int32	u32;
typedef unsigned __int16	u16;
typedef unsigned __int8		u8;
typedef __int64	s64;
typedef __int32	s32;
typedef __int16	s16;
typedef __int8		s8;

#pragma pack(push, 1) 
//#pragma pack(show) 
struct	ether_header {
	u8		ether_dhost[ETHER_ADDR_LEN];
	u8		ether_shost[ETHER_ADDR_LEN];
	u16		ether_type;
};
#pragma pack(pop)
//#pragma pack(show) 
