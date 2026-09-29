OptionsReader.instance().onOptionsReady((options) => {
    if (!options.options_enable_turn_notifications) {
        console.log("Turn notifications disabled.");
        return;
    }
    console.log("Turn notifications enabled.");

    const kMessageType = "awbw_enhancements-turn-state";

    if (typeof Notification === "undefined") {
        console.log("Notifications are not supported in this browser, aborting turn notifications.");
        return;
    }
    if (Notification.permission === "default") {
        Notification.requestPermission();
    }

    function injectTurnWatcher() {
        let s = document.createElement("script");
        s.src = chrome.runtime.getURL("/res/turn_watcher.js");
        (document.head || document.documentElement).appendChild(s);
    }

    function notifyMyTurn(state) {
        if (Notification.permission !== "granted") {
            console.log("Notification permission not granted, skipping turn notification.");
            return;
        }
        if (options.options_turn_notifications_only_when_hidden && !document.hidden) {
            return;
        }

        let title = "AWBW: It's your turn!";
        let body = state.day !== null ? "Day " + state.day : "";
        let notification = new Notification(title, {
            body: body,
            icon: chrome.runtime.getURL("/res/images/fighter_icon128.png"),
            tag: "awbw-turn-" + state.gameId,
        });
        notification.onclick = () => {
            window.focus();
            notification.close();
        };
    }

    // The first state we see is the baseline; only a change to "my turn" afterwards notifies. This
    // avoids a notification just for loading a game that is already waiting on you.
    let previousActivePlayerId = undefined;
    let previousDay = undefined;
    window.addEventListener("message", (event) => {
        if (event.source !== window || !event.data || event.data.type !== kMessageType) {
            return;
        }
        let state = event.data.state;
        if (!state) {
            return;
        }

        let turnChanged = state.activePlayerId !== previousActivePlayerId || state.day !== previousDay;
        let isMyTurn = state.activePlayerId === state.myPlayerId;
        if (previousActivePlayerId !== undefined && turnChanged && isMyTurn) {
            notifyMyTurn(state);
        }
        previousActivePlayerId = state.activePlayerId;
        previousDay = state.day;
    });

    injectTurnWatcher();
});
