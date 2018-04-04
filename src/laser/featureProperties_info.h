#ifndef ED_PLUGIN_FEATUREPROPERTIES_INFO_H_
#define ED_PLUGIN_FEATUREPROPERTIES_INFO_H_

#include <ed/property_info.h>
#include <ed/convex_hull_calc.h>
// #include <problib/include/problib.h>
// #include <problib/pdfs/PMF.h>

class FeaturPropertiesInfo : public ed::PropertyInfo
{

public:

    void serialize ( const ed::Variant& v, ed::io::Writer& w ) const {

        const ed::tracking::FeatureProperties& p = v.getValue<ed::tracking::FeatureProperties>();

        std::string serializedData;
        p.featureProbabilities_.pmf_.serialize ( serializedData );

        w.writeGroup ( "prob" );
        w.writeValue ( "class_data", serializedData );
        w.endGroup();

        w.writeGroup ( "circ" );
        w.writeValue ( "x",     p.circle_.x_ );
        w.writeValue ( "y",     p.circle_.y_ );
        w.writeValue ( "z",     p.circle_.z_ );
        w.writeValue ( "roll",  p.circle_.roll_ );
        w.writeValue ( "pitch", p.circle_.pitch_ );
        w.writeValue ( "yaw",   p.circle_.yaw_ );
        w.endGroup();

        w.writeGroup ( "rec" );
        w.writeValue ( "x",     p.rectangle_.x_ );
        w.writeValue ( "y",     p.rectangle_.y_ );
        w.writeValue ( "z",     p.rectangle_.z_ );
        w.writeValue ( "w",     p.rectangle_.w_ );
        w.writeValue ( "d",     p.rectangle_.d_ );
        w.writeValue ( "h",     p.rectangle_.h_ );
        w.writeValue ( "roll",  p.rectangle_.roll_ );
        w.writeValue ( "pitch", p.rectangle_.pitch_ );
        w.writeValue ( "yaw",   p.rectangle_.yaw_ );
        w.endGroup();
    }

    bool deserialize ( ed::io::Reader& r, ed::Variant& v ) const {

        ed::tracking::FeatureProperties p;
	
	std::string serializedData;
	
        if ( r.readGroup ( "prob" ) ) 
	{
          r.readValue("class_data", serializedData);
        }
        p.featureProbabilities_.pmf_.deserialize( serializedData );
        
	

        if ( r.readGroup ( "circ" ) ) 
        {
            r.readValue ( "x",     p.circle_.x_ );
            r.readValue ( "y",     p.circle_.y_ );
            r.readValue ( "z",     p.circle_.z_ );
            r.readValue ( "roll",  p.circle_.roll_ );
            r.readValue ( "pitch", p.circle_.pitch_ );
            r.readValue ( "yaw",   p.circle_.yaw_ );
            r.endGroup();
        }

        if ( r.readGroup ( "rec" ) ) 
        {
            r.readValue ( "x",     p.rectangle_.x_ );
            r.readValue ( "y",     p.rectangle_.y_ );
            r.readValue ( "z",     p.rectangle_.z_ );
            r.readValue ( "w",     p.rectangle_.w_ );
            r.readValue ( "d",     p.rectangle_.d_ );
            r.readValue ( "h",     p.rectangle_.h_ );
            r.readValue ( "roll",  p.rectangle_.roll_ );
            r.readValue ( "pitch", p.rectangle_.pitch_ );
            r.readValue ( "yaw",   p.rectangle_.yaw_ );
            r.endGroup();
        }

        v = p;
        return true;
    }

    bool serializable() const {
        return true;
    }

};

#endif
