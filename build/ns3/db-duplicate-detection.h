/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Haoliang Chen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Haoliang Chen  <chl41993@gmail.com>
 */

#ifndef DB_DUPLICATE_DETECTION_H
#define DB_DUPLICATE_DETECTION_H
#include <list>
#include <unordered_map>

namespace ns3{
namespace db{

class Duplicate_Detection
{
public:
  Duplicate_Detection ();

  bool CheckThis (uint16_t messageSequenceNumber);
  void SetSize (uint32_t size);

private:
  uint32_t m_size;
  std::list<uint16_t> m_container;
  std::unordered_map<uint16_t, std::list<uint16_t>::iterator > m_containerMap;
};


}
}


#endif //DB_DUPLICATE_DETECTION_H
