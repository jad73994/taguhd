#include <string>
#include <boost/lexical_cast.hpp>
#include <uhd/usrp/mmimo/txrx_net.hpp>
#include <sys/types.h>
#include <ifaddrs.h>

#include <iostream>
#include <boost/format.hpp>

using namespace boost::asio;
using namespace uhd::mmimo;

const unsigned int txrx_net::MAX_BUF_LEN = 1300;
//const unsigned int txrx_net::LISTEN_PORT = 50002;
const unsigned int txrx_net::SENTINEL_ID = 99999;

void txrx_net::create_socket(boost::asio::ip::udp::socket &sock, unsigned int port) {

  sock.open(boost::asio::ip::udp::v4());
  sock.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port));

  sock.set_option(socket_base::reuse_address(true)); // so_reuseaddr
  sock.set_option(socket_base::linger(true, 0));     // so_linger
  sock.set_option(socket_base::receive_buffer_size(32678)); 

}

// std::size_t txrx_net::send_to(boost::asio::ip::udp::socket &sock, const boost::asio::ip::udp::endpoint &dst_ep, std::vector<unsigned char> &buf, bool clear_buff) {
//   //std::cerr << boost::format("Sending message type %u with size %u") % (unsigned int)buf[0] % buf.size() << std::endl;
//   sock.send_to(buffer(buf), dst_ep);
//   std::size_t bufsiz = buf.size();
//   if (clear_buff) {
//     buf.clear();
//   }
//   return bufsiz;
// }
	    
std::string txrx_net::msg_type_to_string(txrx_net::txrx_net_constants_type_t t) {

  std::string s = "";

  switch (t) {
  case txrx_net::NO_MESSAGE: 
    s = "NO_MESSAGE"; break;
  case txrx_net::MEASURE_CFO: 
    s = "MEASURE_CFO"; break;
  case txrx_net::MEASURE_CFO_DONE: 
    s = "MEASURE_CFO_DONE"; break;
  case txrx_net::MEASURE_H:
    s = "MEASURE_H"; break;
  case txrx_net::MEASURED_H:
    s = "MEASURED_H"; break;
  case txrx_net::LOG_PACKET:
    s = "LOG_PACKET"; break;
  case txrx_net::DONT_CARE: 
    s = "DONT_CARE"; break;
  default:
    s = (std::string("Unknown type ") + boost::lexical_cast<std::string>((unsigned int)t)); break;
  }

  if (t & ACK_FLAG) {
    s = s + "|ACK";
  }

  return s;
}

void txrx_net::assemble_buff(txrx_net::txrx_net_constants_type_t message_type, unsigned int srcid, unsigned int dstid, unsigned char *payload, unsigned int payload_size, std::vector<unsigned char> &buff) {

  // format: 
  // 1. message type (1 byte)
  // 2. srcid (4 bytes)
  // 3. dstid (4 bytes)
  // 5. payload (payload_size bytes)

  unsigned int buff_size = sizeof(unsigned char) + sizeof(unsigned int) + sizeof(unsigned int) + payload_size;
  if (buff_size > txrx_net::MAX_BUF_LEN) {
    std::cerr << boost::format("assemble_buff: buff_size %u exceeds maximum %u") % buff_size % txrx_net::MAX_BUF_LEN << std::endl;
    exit(1);
  }

  buff.clear();
  unsigned char mtype = (unsigned char)message_type;
  buff.insert(buff.end(), &mtype, &mtype+1); //message type
  buff.insert(buff.end(), (unsigned char *)(&srcid), (unsigned char *)((&srcid)+1)); //srcid
  buff.insert(buff.end(), (unsigned char *)(&dstid), (unsigned char *)((&dstid)+1)); //dstid
  buff.insert(buff.end(), payload, payload+payload_size); // payload

  //std::cout << boost::format("assemble_buff: message_type=%u, srcid=%u, dstid=%u, size=%u") % (unsigned int)message_type % srcid % dstid % buff.size() << std::endl;
}

void txrx_net::patch_buff_dstid(std::vector<unsigned char> &buff, unsigned int dstid) {
  unsigned int dstid_entry_start = sizeof(unsigned char) + sizeof(unsigned int);
  std::copy((unsigned char *)&dstid, (unsigned char *)(&dstid+1), &buff[dstid_entry_start]);
}

txrx_net::txrx_net_constants_type_t txrx_net::parse_buff(std::vector<unsigned char> &buff, const unsigned int buff_size, const txrx_net::txrx_net_constants_type_t message_type, 
							 unsigned int &srcid, unsigned int dstid, unsigned int &payload_size, unsigned char * &payload) {
  
  unsigned int buff_index = 0, elt_size = 0;
  txrx_net::txrx_net_constants_type_t rx_message_type;
  unsigned int rx_srcid, rx_dstid;

  unsigned char mtype;
  elt_size = sizeof(unsigned char); std::copy(&buff[buff_index], (&buff[buff_index])+elt_size, (unsigned char *)(&mtype)); buff_index += elt_size;
  elt_size = sizeof(unsigned int); std::copy(&buff[buff_index], (&buff[buff_index])+elt_size, (unsigned char *)(&rx_srcid)); buff_index += elt_size;
  elt_size = sizeof(unsigned int); std::copy(&buff[buff_index], (&buff[buff_index])+elt_size, (unsigned char *)(&rx_dstid)); buff_index += elt_size;
  rx_message_type = (txrx_net_constants_type_t)mtype;

  if (((message_type != txrx_net::DONT_CARE) && (rx_message_type != message_type)) || 
      ((srcid != txrx_net::SENTINEL_ID) && (rx_srcid != srcid)) || 
      ((dstid != txrx_net::SENTINEL_ID) && (rx_dstid != dstid))) {
    std::cerr << boost::format("parse_buff: Message mismatch [expected, actual]: message_type [%s,%s], srcid [%u, %u], dstid [%u, %u], size=%u") 
      % txrx_net::msg_type_to_string(message_type).c_str() % txrx_net::msg_type_to_string(rx_message_type).c_str()
      % srcid % rx_srcid % dstid % rx_dstid % buff_size
	      << std::endl;
    exit(1);
  }

  // if (payload_size < (buff_size - buff_index)) {
  //   std::cerr << boost::format("parse_buff: payload buffer too small: required=%u, actual=%u") % (buff_size-buff_index) % payload_size << std::endl;
  //   exit(1);
  // }
  
  srcid = rx_srcid;
  payload_size = buff_size - buff_index;
  payload = &buff[buff_index];

  //elt_size = payload_size*sizeof(unsigned char); std::copy(&buff[buff_index], (&buff[buff_index])+elt_size, payload); buff_index += elt_size;
  return rx_message_type;
}


void txrx_net::get_my_ip(std::string &ipaddr) {
  struct ifaddrs *ifap;
  if (getifaddrs(&ifap) == 0){
    for (struct ifaddrs *iter = ifap; iter != NULL; iter = iter->ifa_next){
      //ensure that the entries are valid
      if (iter->ifa_addr == NULL) continue;
      if (iter->ifa_addr->sa_family != AF_INET) continue;
      if (iter->ifa_netmask->sa_family != AF_INET) continue;
      if (iter->ifa_broadaddr->sa_family != AF_INET) continue;
      if (std::string(iter->ifa_name) != std::string("eth0")) continue;
      
      //append a new set of interface addresses
      boost::asio::ip::address_v4 res = boost::asio::ip::address_v4(ntohl(reinterpret_cast<sockaddr_in*>((iter->ifa_addr))->sin_addr.s_addr));
      ipaddr = res.to_string();       
      if (ipaddr != ip::address_v4::loopback().to_string()) {
        break;     
      }
    }
    freeifaddrs(ifap);
  }  
}
