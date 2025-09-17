import serial
import WaterSurfaceScanner
import CameraCart

def main():
    WSS = WaterSurfaceScanner.WaterSurfaceScanner()
    CC = CameraCart.CameraCart()
    WSS.start_recording() # This starts alternate threads so it can still record while being blocked in the main script
    CC.jog_absolute(14500) # This is blocking 
    WSS.stop_recording()
    WSS.define_transforms()
    WSS.add_transforms()
    data = WSS.get_water_surface_data()
    


if __name__ == "__main__":
    main()
