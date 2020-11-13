import React from 'react';
import { useDispatch } from 'react-redux';
import Button from 'react-bootstrap/Button';
import ButtonGroup from 'react-bootstrap/ButtonGroup';

import PropTypes from 'prop-types';
import {
    externalTriggerToggled,
    triggerStop,
} from '../../../actions/deviceActions';
import { SINGLE, CONTINUOUS, EXTERNAL } from './constants';

const TriggerModeGroup = ({
    triggerMode,
    setTriggerMode,
    hasExternal,
    externalTrigger,
    rttRunning,
}) => {
    const dispatch = useDispatch();
    const setSingleTriggerMode = () => {
        if (triggerMode === CONTINUOUS) {
            dispatch(triggerStop());
        }
        setTriggerMode(SINGLE);
        if (externalTrigger) {
            dispatch(externalTriggerToggled(false));
        }
    };

    const setContinuousTriggerMode = () => {
        setTriggerMode(CONTINUOUS);
        if (externalTrigger) {
            dispatch(externalTriggerToggled(false));
        }
    };

    const setExternalTriggerMode = () => {
        setTriggerMode(EXTERNAL);
        dispatch(externalTriggerToggled(true));
    };
    return (
        <ButtonGroup className="mb-2 w-100 trigger-mode d-flex flex-row">
            <Button
                title="Sample once"
                disabled={!rttRunning || triggerMode === SINGLE}
                variant={triggerMode === SINGLE ? 'set' : 'unset'}
                onClick={setSingleTriggerMode}
            >
                Single
            </Button>
            <Button
                title="Sample until stopped by user"
                disabled={!rttRunning || triggerMode === CONTINUOUS}
                variant={triggerMode === CONTINUOUS ? 'set' : 'unset'}
                onClick={setContinuousTriggerMode}
            >
                Continuous
            </Button>
            {hasExternal && (
                <Button
                    title="Sample controlled from TRIG IN"
                    disabled={!rttRunning || triggerMode === EXTERNAL}
                    variant={triggerMode === EXTERNAL ? 'set' : 'unset'}
                    onClick={setExternalTriggerMode}
                >
                    External
                </Button>
            )}
        </ButtonGroup>
    );
};

export default TriggerModeGroup;

TriggerModeGroup.propTypes = {
    triggerMode: PropTypes.string.isRequired,
    setTriggerMode: PropTypes.func.isRequired,
    hasExternal: PropTypes.bool.isRequired,
    externalTrigger: PropTypes.bool.isRequired,
    rttRunning: PropTypes.bool.isRequired,
};
