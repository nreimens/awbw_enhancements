// Runs in the page's own JS context (injected by turn_notifications_script.js) so it can read the
// game page's global variables, which content scripts cannot see.
//
// NOTE: the globals below are the page variables this was written against. If AWBW renames them,
// readTurnState() is the only function that needs updating.
(function () {
    const kPollIntervalMs = 3000;
    const kMessageType = "awbw_enhancements-turn-state";

    function readTurnState() {
        let activePlayerId = window.currentTurn;
        let myPlayerId = window.playerId;
        if (activePlayerId === undefined || activePlayerId === null || myPlayerId === undefined || myPlayerId === null) {
            return null;
        }

        let day = window.gameDay !== undefined ? window.gameDay : null;
        return {
            gameId: window.gameId !== undefined ? window.gameId : null,
            day: day,
            activePlayerId: "" + activePlayerId,
            myPlayerId: "" + myPlayerId,
        };
    }

    function poll() {
        let state = null;
        try {
            state = readTurnState();
        } catch (e) {
            console.log("AWBW Enhancements: failed to read turn state:", e);
        }
        window.postMessage({type: kMessageType, state: state}, window.location.origin);
    }

    poll();
    setInterval(poll, kPollIntervalMs);
})();
