#!/usr/bin/env ruby

require_relative 'bmp085'

epoch = Time.now.to_i
sensor = I2CDevice::BMP085.new

puts "temperature\t#{sensor.read_temperature}\t#{epoch}"
puts "pressure\t#{sensor.read_pressure / 100.0}\t#{epoch}"
