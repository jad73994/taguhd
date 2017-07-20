#
# Copyright 2010-2011 Ettus Research LLC
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

########################################################################
# mmimo applications
########################################################################
SET(mmimo_sources
  general_tx_rx.cpp
  # tx_samples_from_file_mimo_2x_auto_nw.cpp
  # rx_samples_to_file_2x_auto_nw.cpp
  # send_packet.cpp
  # ofdm_mimo_tx.cpp
  # wait_for_packet.cpp
)

#for each source: build an executable and install
FOREACH(mmimo_source ${mmimo_sources})
    GET_FILENAME_COMPONENT(mmimo_name ${mmimo_source} NAME_WE)
    ADD_EXECUTABLE(${mmimo_name} ${mmimo_source})
    TARGET_LINK_LIBRARIES(${mmimo_name} uhd)
    INSTALL(TARGETS ${mmimo_name} RUNTIME DESTINATION ${PKG_DATA_DIR}/mmimo COMPONENT mmimo)
ENDFOREACH(mmimo_source)
