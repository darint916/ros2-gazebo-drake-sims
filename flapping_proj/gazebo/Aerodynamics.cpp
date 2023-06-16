class Aerodynmaics 
{
    void OdometryState::Configure(const igz::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, igz::EntityComponentManager &_ecm, igz::EventManager &/*_eventMgr*/)
    {
        this->dataPtr->model = igz::Model(_entity);
        if(!this->dataPtr->model.Valid(_ecm)) {
            ignerr << "plugin should be attached to a model entity, fail init" << std::endl;
            return;
        }
    }
}