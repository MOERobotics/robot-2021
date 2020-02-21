package frc.robot;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.locks.ReentrantLock;

public class Logger {

	public static final Logger instance = new Logger();
	protected Logger() {} //No touch.

	/**Logs a message to STDOUT. Does not check any previous calls.*/
	public static void log(String key, String message) {
		instance.logInternal(key,message,null,null, true);
	}
	/**Logs a message to STDOUT. Will prevent all future calls to log.*/
	public static void logOnce(String key, String message) {
		instance.logInternal(key,message,null,null, false);
	}
	/**Logs a message to STDOUT. Will prevent all logged messages for N milliseconds.*/
	public static void logTTL(String key, String message, int ttl_ms) {
		instance.logInternal(key,message,null,ttl_ms, false);
	}
	/**Logs a message to STDOUT. Will prevent all logged messages until value changes.*/
	public static void logValue(String key, String message, long value) {
		instance.logInternal(key,message,Long.hashCode(value),null, false);
	}
	/**Logs a message to STDOUT. Will prevent all logged messages until value changes.*/
	public static void logValue(String key, String message, double value) {
		instance.logInternal(key,message,Double.hashCode(value),null, false);
	}
	/**Logs a message to STDOUT. Will prevent all logged messages until value changes.*/
	public static void logValue(String key, String message, Object value) {
		instance.logInternal(key,message, Objects.hashCode(value),null, false);
	}
	/**Logs a message to STDOUT. Will prevent all logged messages for N milliseconds, or until value changes.*/
	public static void logValueTTL(String key, String message, long value, int ttl_ms) {
		instance.logInternal(key,message,Long.hashCode(value),ttl_ms, false);
	}
	/**Logs a message to STDOUT. Will prevent all logged messages for N milliseconds, or until value changes.*/
	public static void logValueTTL(String key, String message, double value, int ttl_ms) {
		instance.logInternal(key,message,Double.hashCode(value),ttl_ms, false);
	}
	/**Logs a message to STDOUT. Will prevent all logged messages for N milliseconds, or until value changes.*/
	public static void logValueTTL(String key, String message, Object value, int ttl_ms) {
		instance.logInternal(key,message, Objects.hashCode(value),ttl_ms, false);
	}


	private static class LoggedMessage {
		final String        key;
		final String        message;
		final Integer       hash;
		final LocalDateTime expiryTime;
		LoggedMessage(
				String key,
				String message,
				Integer hash,
				LocalDateTime expiryTime
		) {
			this.key = key;
			this.message = message;
			this.hash = hash;
			this.expiryTime = expiryTime;
		}

		@Override
		public String toString() {
			return String.format(
					"%16.16s | %8.8s | %12.12s | %s",
					key,
					hashString(),
					expiryTimeString(),
					message
			);
		}

		String hashString() {
			return (hash == null)
					? "--NULL--"
					: String.format("%08X", hash);
		}


		static final DateTimeFormatter timeFormatter
				= DateTimeFormatter.ofPattern("HH:mm:ss.SSS");

		String expiryTimeString() {
			return (expiryTime == null)
					? "Does Not Expire"
					: timeFormatter.format(expiryTime);
		}

	}

	private static final LoggedMessage NullLoggedMessage = new LoggedMessage(
			"NULL",
			"NULL",
			null,
			null
	);


	private ReentrantLock messageLock = new ReentrantLock();
	private Map<String, LoggedMessage> loggedMessages = new HashMap<>();

	private void logInternal(
			String key,
			String message,
			Integer hash,
			Integer ttl_ms,
			boolean ignoreChecks
	) {
		var now = LocalDateTime.now();
		LocalDateTime expiryTime = null;
		if (ttl_ms != null) {
			long ttl_ns = ttl_ms * 1000000;
			expiryTime = now.plusNanos(ttl_ns);
		}
		var newMessage = new LoggedMessage(
				key,
				message,
				hash,
				expiryTime
		);
		try {
			messageLock.lock();
			if (!loggedMessages.containsKey(key)) {
				loggedMessages.put(key,NullLoggedMessage);
			}

			var oldMessage = loggedMessages.get(key);

			System.err.printf("old key:'%s' value:'%s' \n", oldMessage.key, oldMessage.hashString());
			System.err.printf("new key:'%s' value:'%s' \n", newMessage.key, newMessage.hashString());
			System.err.printf("KEY: '%s' VALUE: '%s' DATE: '%s'\n",
					Boolean.toString(!Objects.equals(oldMessage.key, newMessage.key)),
					Boolean.toString(
							oldMessage.hash != null &&
									newMessage.hash != null &&
									oldMessage.hash.intValue() == newMessage.hash.intValue()
					),
					Boolean.toString(
							newMessage.expiryTime != null &&
									oldMessage.expiryTime != null &&
									oldMessage.expiryTime.isBefore(LocalDateTime.now())
					)
			);
			if (
					ignoreChecks == true ||
							checkMessageExpired(oldMessage,newMessage) == true
			) {
				System.out.printf(
						"LOGGER: %12.12s | %16.16s | %8.8s | %s\n",
						LoggedMessage.timeFormatter.format(now),
						key,
						newMessage.hashString(),
						message
				);
				loggedMessages.put(key,newMessage);
			}

		} finally {
			messageLock.unlock();
		}
	}

	private static boolean checkMessageExpired (
			LoggedMessage oldMsg,
			LoggedMessage newMsg
	) {
		//NULL message case
		if (!Objects.equals(oldMsg.key, newMsg.key)) return true;
		//Changed value case
		if (
				oldMsg.hash != null &&
						newMsg.hash != null &&
						oldMsg.hash.intValue() == newMsg.hash.intValue()
		) return true;
		//Datetime expired case
		if (
				newMsg.expiryTime != null &&
				oldMsg.expiryTime != null &&
						oldMsg.expiryTime.isBefore(LocalDateTime.now())
		) return true;
		int i = 1;
		//Default
		return false;
	}


}
