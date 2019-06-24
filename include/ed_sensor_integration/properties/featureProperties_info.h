#ifndef ED_PLUGIN_FEATUREPROPERTIES_INFO_H_
#define ED_PLUGIN_FEATUREPROPERTIES_INFO_H_

#include <ed/property_info.h>
//#include <ed>
#include "../../../src/laser/feature_functions.h"
//#include <ed_laser_plugin_tracking/feature_functions.h>

// #include <problib/include/problib.h>
// #include <problib/pdfs/PMF.h>

class FeaturPropertiesInfo : public ed::PropertyInfo
{

public:

    void serialize ( const ed::Variant& v, ed::io::Writer& w ) const {

        const ed::tracking::FeatureProperties& p = v.getValue<ed::tracking::FeatureProperties>();

        std::string serializedData;
        p.featureProbabilities_.pmf_->serialize ( serializedData );

        w.writeGroup ( "prob" );
        w.writeValue ( "class_data", serializedData );
        w.endGroup();

        w.writeGroup ( "circ" );        
        w.writeValue ( "x",     p.getCircle().get_x()  );
        w.writeValue ( "y",     p.getCircle().get_y() );
        w.writeValue ( "z",     p.getCircle().get_z() );
        w.writeValue ( "roll",  p.getCircle().get_roll() );
        w.writeValue ( "pitch", p.getCircle().get_pitch() );
        w.writeValue ( "yaw",   p.getCircle().get_yaw() );
        w.endGroup();

        w.writeGroup ( "rec" );
        w.writeValue ( "x",     p.getRectangle().get_x() );
        w.writeValue ( "y",     p.getRectangle().get_y() );
        w.writeValue ( "z",     p.getRectangle().get_z() );
        w.writeValue ( "w",     p.getRectangle().get_w() );
        w.writeValue ( "d",     p.getRectangle().get_d() );
        w.writeValue ( "h",     p.getRectangle().get_h() );
        w.writeValue ( "roll",  p.getRectangle().get_roll() );
        w.writeValue ( "pitch", p.getRectangle().get_pitch() );
        w.writeValue ( "yaw",   p.getRectangle().get_yaw() );
        w.writeValue ( "nMeasurements",  p.getNMeasurements() );
        w.endGroup();
    }

    bool deserialize ( ed::io::Reader& r, ed::Variant& v ) const {

        ed::tracking::FeatureProperties p;
	
	std::string serializedData;
	
        if ( r.readGroup ( "prob" ) ) 
	{
          r.readValue("class_data", serializedData);
        }
        p.featureProbabilities_.pmf_->deserialize( serializedData );
       
        float x, y, z, w, d, h, roll, pitch, yaw;
        int nMeasurements;
        
        if ( r.readGroup ( "circ" ) ) 
        {       
                r.readValue ( "x",     x );
                r.readValue ( "y",     y );
                r.readValue ( "z",     z );
                r.readValue ( "roll",  roll );
                r.readValue ( "pitch", pitch );
                r.readValue ( "yaw",   yaw );
            
                p.circle_.set_x(x);
                p.circle_.set_y(y);
                p.circle_.set_z(z);
                p.circle_.set_roll(roll);
                p.circle_.set_pitch(pitch);
                p.circle_.set_yaw(yaw);
            
                r.endGroup();
        }

        if ( r.readGroup ( "rec" ) ) 
        {
                r.readValue ( "x",     x );
                r.readValue ( "y",     y );
                r.readValue ( "z",     z );
                r.readValue ( "w",     w );
                r.readValue ( "d",     d );
                r.readValue ( "h",     roll );
                r.readValue ( "roll",  pitch );
                r.readValue ( "pitch", yaw );
            
                r.readValue ( "yaw",   yaw );
                r.readValue ( "nMeasurements",   nMeasurements );
                r.endGroup();
                
                p.rectangle_.set_x(x);
                p.rectangle_.set_y(y);
                p.rectangle_.set_z(z);
                p.rectangle_.set_w(w);
                p.rectangle_.set_d(d);
                p.rectangle_.set_h(h);
                p.rectangle_.set_roll(roll);
                p.rectangle_.set_pitch(pitch);
                p.rectangle_.set_yaw(yaw);
                p.setNMeasurements( nMeasurements );
        }

        v = p;
        return true;
    }

    bool serializable() const {
        return true;
    }

};

#endif
