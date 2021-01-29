/**
 * Copyright (C) 2021 https://github.com/d3vf1x
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
void siginthandler(int param);

int read_data_from_sensor();

int send_data_to_display(const void *buf, size_t n);

void close_fds();

void strip_clear();

void strip_update();

void strip_set_led(uint8_t nr, uint8_t r, uint8_t g, uint8_t b);