/*
 * Author: Evan Beachly
 * Date: September 4, 2015
 * Description: Code for performing non-blocking IO over a Serial Port
 */

#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <boost/asio.hpp>
#include <memory>
#include <boost/thread.hpp>
#include <stdint.h>

class Serial {
private:
    std::auto_ptr<boost::thread>            serialReader;
    std::auto_ptr<boost::asio::io_service>  io_service;
    std::auto_ptr<boost::asio::serial_port> serial_port;

    //A circular byte buffer
    uint8_t     byteBuffer[65536];
    uint16_t    pushPtr;
    uint16_t    popPtr;
    uint32_t    size;
    bool        alive;
    std::string error;
    boost::mutex mtx;

    // Callback when a byte has been read
    void readHandler(
            const boost::system::error_code& errorCode, // Result of operation.
            std::size_t bytes_transferred               // Number of bytes transferred
    )
    {
        if (bytes_transferred == 1) {
            //The byte was already placed into the buffer
            //Lock mutex
            mtx.lock();
            //Increment insert ptr
            ++pushPtr;
            //Check if the buffer is full
            if (size == 65536) {
                //Increment the pop pointer
                ++popPtr;
            } else {
                //Increment the size
                ++size;
            }
            //Unlock mutex
            mtx.unlock();
        }
        else
        {
            //Error occurred
            error = errorCode.message();
            alive = false;
        }
        //If the object is still alive, start another read operation
        if ( alive )
        {
            boost::asio::async_read( *serial_port, boost::asio::buffer(&byteBuffer[pushPtr],1), boost::bind(&Serial::readHandler, this, _1, _2) );
        }
    }

    void readerThread()
    {
        try
        {
            //Start reading
            boost::asio::async_read( *serial_port, boost::asio::buffer(&byteBuffer[pushPtr],1), boost::bind(&Serial::readHandler, this, _1, _2) );
            //While the object still exists
            while (alive)
            {
                //Process I/O
                io_service->run();
            }
        }
        catch (std::exception& e)
        {
            error = e.what();
        }

        alive = false;
    }

public:
    Serial( std::string portFile, int baud )
    {
        pushPtr = 0;
        popPtr = 0;
        size = 0;


        io_service.reset( new boost::asio::io_service() );
        try
        {
            serial_port.reset( new boost::asio::serial_port(*io_service, portFile));
        }
        catch (std::exception& e)
        {
            io_service.reset();
            throw std::runtime_error("Serial: error opening serial port on " + portFile + ": " + e.what());
        }

        serial_port->set_option( boost::asio::serial_port_base::baud_rate(baud));
        serial_port->set_option( boost::asio::serial_port_base::character_size(8) );
        serial_port->set_option( boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none ) );
        serial_port->set_option( boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none ) );
        serial_port->set_option( boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one ) );

        alive = true;
        serialReader.reset( new boost::thread( boost::bind(&Serial::readerThread, this) ));
    }

    ~Serial()
    {
        //Indicate that this is now dead
        alive = false;

        //Abort any operations
        serial_port->cancel();

        //Destroy the serial port and then the io service.
        serial_port.reset();
        io_service.reset();

        //Join the thread
        serialReader->join();
    }

    /*
     * Attempts to read a byte from the serial port. Returns immediately
     * @param byt     A pointer to where to store the byte that is read
     * @returns       true if data was read, false otherwise.
     * @throws        std::runtime_error if the reader has failed.
     */
    bool readByte(uint8_t* byt)
    {
        //lock the mutex
        mtx.lock();
        if (size == 0)
        {
            mtx.unlock();

            //Error check
            if (error.size() > 0)
            {
                throw std::runtime_error("Serial: " + error);
            }
            else if (!alive)
            {
                throw std::runtime_error("Serial: Reader Thread has unexpectedly terminated.");
            }
            return false;
        }
        else
        {
            *byt = byteBuffer[popPtr];
            --size;
            ++popPtr;
            mtx.unlock();
            return true;
        }
    }

    void writeByte(uint8_t byt)
    {
        boost::asio::write(*serial_port, boost::asio::buffer(&byt,1));
    }

    void writeBytes(const uint8_t* bytes, int num )
    {
        boost::asio::write(*serial_port, boost::asio::buffer(bytes,num));
    }

};

#endif

