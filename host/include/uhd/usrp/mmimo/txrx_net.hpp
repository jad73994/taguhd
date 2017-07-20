#ifndef INCLUDED_UHD_USRP_MMIMO_TXRX_NET_HPP
#define INCLUDED_UHD_USRP_MMIMO_TXRX_NET_HPP

//#include <cstdint>
#include <string>
#include <vector>
#include <boost/asio.hpp>
#include <uhd/config.hpp>

namespace uhd {
  namespace mmimo{
    struct UHD_API txrx_net {

      static const unsigned int TX_PORT = 50000;
      static const unsigned int RX_PORT = 50001;
      static const unsigned int MATLAB_PORT = 50002;

      //typedef boost::shared_ptr<boost::asio::ip::udp::socket> socket_sptr;
      static const unsigned int MAX_BUF_LEN;
      // static const unsigned int LISTEN_PORT;
      static const unsigned int SENTINEL_ID;

      static const unsigned int TX_FLAG = 0x00000000; 
      static const unsigned int RX_FLAG = 0x80000000;
      static const unsigned int ID_MASK = 0x7fffffff;

      enum txrx_net_constants_type_t {
	NO_MESSAGE = 0,

	WAKEUP = 1,

	SET_PPS = 2,

	MEASURE_CFO = 3,
	MEASURE_CFO_DONE = 4,

	MEASURE_H = 5,

	MEASURED_H = 6,

	LOG_PACKET = 7,

	DONT_CARE = 127,

	ACK_FLAG = 128,
	
      };
      
      static void create_socket(boost::asio::ip::udp::socket &sock, unsigned int port);
      static void get_my_ip(std::string &ipaddr);
      // static std::size_t send_to(boost::asio::ip::udp::socket &sock, const boost::asio::ip::udp::endpoint &, std::vector<unsigned char> &buf, bool clear_buff = true);
      static std::string msg_type_to_string(txrx_net_constants_type_t);

      static void assemble_buff(txrx_net_constants_type_t message_type, unsigned int srcid, unsigned int dstid, unsigned char *payload, unsigned int payload_size, std::vector<unsigned char> &buff);
      static void patch_buff_dstid(std::vector<unsigned char> &buff, unsigned int dstid);
      static txrx_net_constants_type_t parse_buff(std::vector<unsigned char> &buff, const unsigned int buff_size, const txrx_net_constants_type_t message_type, 
						  unsigned int &srcid, unsigned int dstid, unsigned int &payload_size, unsigned char * &payload);


    };
  }
}

#endif /* INCLUDED_UHD_USRP_MMIMO_TXRX_NET_HPP */
